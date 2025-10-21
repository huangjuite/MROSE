#include "RosNode/MapRosNode.h"
#include "RadarPointType.h"
#include "utils.h"

template <class PointT>
MapOptimize<PointT>::MapOptimize()
    : rclcpp::Node("radar_mapping",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {
  auto cfg =
      get_parameter_or("config", rclcpp::Parameter("config", "config.yaml"));
  params = rplrio::get_params(cfg.as_string());

  path.header.frame_id = "map";
  pub_path = this->create_publisher<nav_msgs::msg::Path>("radar/path", 1);
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
void MapOptimize<PointT>::cb_node(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr radar_msg,
    const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  PclPtr radar_pc(new pcl::PointCloud<PointT>);
  fromROSMsg(*radar_msg, *radar_pc);

  gtsam::Pose3 current_pose(
      gtsam::Rot3(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                  odom->pose.pose.orientation.y, odom->pose.pose.orientation.z),
      gtsam::Point3(odom->pose.pose.position.x, odom->pose.pose.position.y,
                    odom->pose.pose.position.z));

  global_map_mtx.lock();
  pcl::PointCloud<PointT> p;
  pcl::transformPointCloud(*radar_pc, p, current_pose.matrix());
  global_map += p;
  global_map_mtx.unlock();

  path.poses.push_back(rplrio::toROSPoseStamped(current_pose, odom->header));
  path.header.stamp = odom->header.stamp;
  pub_path->publish(path);

  // broadcast transformation
  geometry_msgs::msg::TransformStamped transform_stamped =
      rplrio::toROSTransform(current_pose, "map", "base_link",
                             odom->header.stamp);
  br->sendTransform(transform_stamped);

  // reset
  last_pose = current_pose;
}

template <class PointT> void MapOptimize<PointT>::render_map() {
  rclcpp::Rate rate(params.render_rate);
  while (rclcpp::ok()) {
    rate.sleep();
    sensor_msgs::msg::PointCloud2 map_msg;
    toROSMsg(global_map, map_msg);
    map_msg.header.frame_id = "map";
    pub_map->publish(map_msg);
  }
}

template <class PointT> MapOptimize<PointT>::~MapOptimize() {}

template class MapOptimize<pcl::PointXYZSD>;
template class MapOptimize<pcl::PointXYZSDW>;
template class MapOptimize<pcl::PointHugin>;