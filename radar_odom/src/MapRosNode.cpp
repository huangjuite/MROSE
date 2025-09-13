#include "MapRosNode.h"
#include "utils.h"
#include <memory>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

template <class PointT>
MapOptimize<PointT>::MapOptimize()
    : rclcpp::Node("radar_mapping",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {
  auto cfg =
      get_parameter_or("config", rclcpp::Parameter("config", "config.yaml"));
  params = rplrio::get_params(cfg.as_string());

  // pcl init
  points_cache.push_back(nullptr); // align index with graph
  voxel_filter.setLeafSize(params.process_voxel_size, params.process_voxel_size,
                           params.process_voxel_size);
  snr_filter.setFilterFieldName("snr");
  snr_filter.setFilterLimits(params.snr_thresh, FLT_MAX);

  gicp_reg.setNumThreads(params.pc_reg_thread);
  gicp_reg.setCorrespondenceRandomness(20);
  gicp_reg.setMaxCorrespondenceDistance(1.0);
  // gicp_reg.setVoxelResolution(1.0);
  // gicp_reg.setRegistrationType("VGICP"); // or "GICP" (default = "GICP")

  // gtsam init
  graph = std::make_unique<gtsam::NonlinearFactorGraph>();
  gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(1, 0, 0, 0),
                          gtsam::Point3(0, 0, 0));
  pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
  loop_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.02, 0.02, 0.02).finished());
  graph->addPrior(X(0), prior_pose, pose_noise_model);
  init_values.insert(X(0), prior_pose);
  last_pose = prior_pose;

  // ros init
  loop_marker.header.frame_id = "map";
  loop_marker.action = visualization_msgs::msg::Marker::ADD;
  loop_marker.ns = "loop_detected";
  loop_marker.id = 0;
  loop_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  loop_marker.pose = rplrio::toROSPose(gtsam::Pose3::Identity());
  loop_marker.scale.x = 0.1;
  loop_marker.color.a = 1;
  loop_marker.color.r = 1;
  path.header.frame_id = "map";

  pub_local =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 1);
  pub_subm =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_map", 1);
  pub_aligned =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_map", 1);

  pub_path = this->create_publisher<nav_msgs::msg::Path>("radar_path", 1);
  pub_loop_marker = this->create_publisher<visualization_msgs::msg::Marker>(
      "loop_detected", 1);
  pub_map =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("radar/map", 1);

  br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  rclcpp::QoS qos = rclcpp::QoS(10);
  sub_radar.subscribe(this, "radar_inlier", qos.get_rmw_qos_profile());
  sub_odom.subscribe(this, "dopplerIO", qos.get_rmw_qos_profile());
  sync = std::make_unique<message_filters::Synchronizer<MySyncPolicy>>(
      MySyncPolicy(10), sub_radar, sub_odom);
  sync->registerCallback(std::bind(&MapOptimize::cb_node, this,
                                   std::placeholders::_1,
                                   std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "radar pose graph init");
}

template <class PointT>
std::pair<int, int> MapOptimize<PointT>::detect_loop(gtsam::Values &nodes,
                                                     uint step) {
  std::pair<int, int> close_pairs{step, -1};

  if (step < params.loop_ignore)
    return close_pairs;

  double min_dis = 1e4;
  gtsam::Pose3 head_pose = nodes.at<gtsam::Pose3>(X(step));

  for (uint i = step - params.loop_ignore; i > params.key_frame_search; i--) {
    gtsam::Pose3 node_pose = nodes.at<gtsam::Pose3>(X(i));
    double dis =
        gtsam::distance3(node_pose.translation(), head_pose.translation());
    if (dis < params.loop_distance_thresh && dis < min_dis) {
      close_pairs.second = i;
      min_dis = dis;
    }
  }

  return close_pairs;
}

template <class PointT>
typename pcl::PointCloud<PointT>::Ptr
MapOptimize<PointT>::build_submap(uint step, uint start, uint end) {
  PclPtr smap(new pcl::PointCloud<PointT>);
  smap->header.frame_id = "map";
  gtsam::Pose3 cp = init_values.at<gtsam::Pose3>(X(step));
  for (uint i = start; i <= end; i++) {
    gtsam::Pose3 t = cp * cp.between(init_values.at<gtsam::Pose3>(X(i)));
    pcl::PointCloud<PointT> p;
    pcl::transformPointCloud(*points_cache[i], p, t.matrix());
    *smap += p;
  }

  voxel_filter.setInputCloud(smap);
  voxel_filter.filter(*smap);
  snr_filter.setInputCloud(smap);
  snr_filter.filter(*smap);

  return smap;
}

template <class PointT>
void MapOptimize<PointT>::cb_node(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr radar_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  PclPtr radar_pc(new pcl::PointCloud<PointT>);
  fromROSMsg(*radar_msg, *radar_pc);
  rplrio::logdB_SNR(radar_pc);

  // voxel_filter.setInputCloud(radar_pc);
  // voxel_filter.filter(*radar_pc);
  // snr_filter.setInputCloud(radar_pc);
  // snr_filter.filter(*radar_pc);

  points_cache.push_back(radar_pc);

  gtsam::Pose3 current_pose(
      gtsam::Rot3(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z),
      gtsam::Point3(odom->pose.pose.position.x, odom->pose.pose.position.y,
                    odom->pose.pose.position.z));

  global_map_mtx.lock();
  step++;
  gtsam::Pose3 tail_pose = init_values.at<gtsam::Pose3>(X(step - 1));
  tail_pose = tail_pose * last_pose.between(current_pose);
  path.poses.push_back(rplrio::toROSPoseStamped(tail_pose, odom->header));
  init_values.insert(X(step), tail_pose);
  pcl::PointCloud<PointT> p;
  pcl::transformPointCloud(*radar_pc, p, current_pose.matrix());
  global_map += p;
  global_map_mtx.unlock();

  auto noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 2e-2, 2e-2, 2e-2).finished());
  graph->add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(step - 1), X(step), last_pose.between(current_pose), noise));

  // loop detect
  uint detect_point =
      (step > params.key_frame_search) ? step - params.key_frame_search : 0;
  if (params.enable_loop_close &&
      detect_point - last_loop_detect > params.loop_close_rate) {
    auto l = detect_loop(init_values, detect_point);
    if (l.second != -1) {
      last_loop_detect = detect_point;
      a_loop_closed = true;

      // build submap
      PclPtr local_map =
          build_submap(l.first, l.first - params.key_frame_search,
                       l.first + params.key_frame_search);
      PclPtr sub_map =
          build_submap(l.second, l.second - params.key_frame_search,
                       l.second + params.key_frame_search);

      // find transform
      pcl::PointCloud<PointT> aligned_map;
      gicp_reg.setInputSource(local_map);
      gicp_reg.setInputTarget(sub_map);
      gicp_reg.align(aligned_map);
      Eigen::Matrix4f tf = gicp_reg.getFinalTransformation();

      if (gicp_reg.getFitnessScore() < 1) {
        RCLCPP_INFO(get_logger(), "loop detected, score %.4f",
                    gicp_reg.getFitnessScore());
        loop_cache.push_back(l);

        // loop close on 2d only
        gtsam::Pose3 t(tf.cast<double>());
        gtsam::Pose3 tf3(gtsam::Rot3::Yaw(t.rotation().yaw()),
                         gtsam::Point3(t.translation().x(), t.translation().y(),
                                       t.translation().z()));
        gtsam::Pose3 correct_tf =
            tf3 * init_values.at<gtsam::Pose3>(X(l.first));
        gtsam::Pose3 between_tf =
            correct_tf.between(init_values.at<gtsam::Pose3>(X(l.second)));
        graph->add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(l.first), X(l.second), between_tf, loop_noise_model));

        // optimize
        gtsam::LevenbergMarquardtOptimizer optimizer(*graph, init_values);
        init_values = optimizer.optimize();

        // update visualization
        for (uint i = 0; i < step; i++) {
          path.poses[i].pose =
              rplrio::toROSPose(init_values.at<gtsam::Pose3>(X(i + 1)));
        }

        loop_marker.points.clear();
        for (auto p : loop_cache) {
          gtsam::Point3 p1 =
              init_values.at<gtsam::Pose3>(X(p.first)).translation();
          gtsam::Point3 p2 =
              init_values.at<gtsam::Pose3>(X(p.second)).translation();
          loop_marker.points.push_back(rplrio::toROSPoint(p1));
          loop_marker.points.push_back(rplrio::toROSPoint(p2));
        }
        pub_loop_marker->publish(loop_marker);
      } else {
        RCLCPP_INFO(get_logger(), "loop denied, score %.4f",
                    gicp_reg.getFitnessScore());
      }

      // points visualization
      sensor_msgs::msg::PointCloud2 msg;
      toROSMsg(*local_map, msg);
      pub_local->publish(msg);
      toROSMsg(*sub_map, msg);
      pub_subm->publish(msg);
      toROSMsg(aligned_map, msg);
      pub_aligned->publish(msg);
    }
  }

  // visualize poses
  path.header.stamp = odom->header.stamp;
  pub_path->publish(path);

  // broadcast transformation
  geometry_msgs::msg::TransformStamped transform_stamped =
      rplrio::toROSTransform(tail_pose, "map", "base", odom->header.stamp);
  br->sendTransform(transform_stamped);

  // reset
  last_pose = current_pose;
}

template <class PointT> MapOptimize<PointT>::~MapOptimize() {}

template <class PointT> void MapOptimize<PointT>::publishMap() {
  if (a_loop_closed) {
    std::lock_guard<std::mutex> lock(global_map_mtx);
    global_map.clear();

    for (uint i = 1; i <= step; i++) {
      gtsam::Pose3 t = init_values.at<gtsam::Pose3>(X(i));
      pcl::PointCloud<PointT> p;
      pcl::transformPointCloud(*points_cache[i], p, t.matrix());
      global_map += p;
    }

    a_loop_closed = false;
  }
  sensor_msgs::msg::PointCloud2 map_msg;
  toROSMsg(global_map, map_msg);
  map_msg.header.frame_id = "map";
  pub_map->publish(map_msg);
}

template <class PointT> void MapOptimize<PointT>::render_map() {
  rclcpp::Rate rate(params.render_rate);
  while (rclcpp::ok()) {
    rate.sleep();
    publishMap();
  }
}

template class MapOptimize<pcl::PointXYZSD>;
template class MapOptimize<pcl::PointXYZSDW>;