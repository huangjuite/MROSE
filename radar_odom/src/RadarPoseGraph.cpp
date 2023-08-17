#include "RadarPoseGraph.h"

RadarPoseGraph::RadarPoseGraph(ros::NodeHandle *nh)
{
    // pcl init
    points_cache.push_back(nullptr); // align index with graph
    voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    snr_filter.setFilterFieldName("snr");
    snr_filter.setFilterLimits(50, FLT_MAX);
    ndt_omp.setResolution(1);
    ndt_omp.setNumThreads(8);
    ndt_omp.setNeighborhoodSearchMethod(pclomp::DIRECT7);

    // gtsam init
    // graph = new NonlinearFactorGraph();
    graph = std::make_unique<gtsam::NonlinearFactorGraph>();
    gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(1, 0, 0, 0), gtsam::Point3(0, 0, 0));
    pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
    loop_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.02, 0.02, 0.02).finished());
    graph->addPrior(X(0), prior_pose, pose_noise_model);
    init_values.insert(X(0), prior_pose);
    last_pose = prior_pose;

    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.0;
    // parameters.relinearizeSkip = 1;
    // isam2 = std::make_unique<gtsam::ISAM2>(parameters);

    // ros init
    loop_marker.header.frame_id = "map";
    loop_marker.action = visualization_msgs::Marker::ADD;
    loop_marker.ns = "loop_detected";
    loop_marker.id = 0;
    loop_marker.type = visualization_msgs::Marker::LINE_LIST;
    loop_marker.pose = toROSPose(gtsam::Pose3::Identity());
    loop_marker.scale.x = 0.1;
    loop_marker.color.a = 1;
    loop_marker.color.r = 1;
    path.header.frame_id = "map";

    pub_local = nh->advertise<sensor_msgs::PointCloud2>("local_map", 1);
    pub_subm = nh->advertise<sensor_msgs::PointCloud2>("sub_map", 1);
    pub_aligned = nh->advertise<sensor_msgs::PointCloud2>("aligned_map", 1);

    pub_path = nh->advertise<nav_msgs::Path>("radar_path", 1);
    pub_loop_marker = nh->advertise<visualization_msgs::Marker>("loop_detected", 1);
    sub_radar.subscribe(*nh, "radar_inlier", 1);
    sub_odom.subscribe(*nh, "dopplerIO", 1);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_radar, sub_odom);
    sync->registerCallback(boost::bind(&RadarPoseGraph::cb_node, this, _1, _2));
    ROS_INFO("radar pose graph init");
}

std::pair<int, int> RadarPoseGraph::detect_loop(gtsam::Values &nodes, uint step)
{
    std::pair<int, int> close_pairs{step, -1};

    if (step < loop_ignore)
        return close_pairs;

    double min_dis = 1e4;
    gtsam::Pose3 head_pose = nodes.at<gtsam::Pose3>(X(step));

    for (uint i = step - loop_ignore; i > key_frame_search; i--)
    {
        gtsam::Pose3 node_pose = nodes.at<gtsam::Pose3>(X(i));
        double dis = gtsam::distance3(node_pose.translation(), head_pose.translation());
        if (dis < loop_distance_thresh && dis < min_dis)
        {
            close_pairs.second = i;
            min_dis = dis;
        }
    }

    return close_pairs;
}

pcl::PointCloud<pcl::PointXYZSD>::Ptr RadarPoseGraph::build_submap(uint step, uint start, uint end)
{
    pcl::PointCloud<pcl::PointXYZSD>::Ptr smap(new pcl::PointCloud<pcl::PointXYZSD>);
    smap->header.frame_id = "map";
    gtsam::Pose3 cp = init_values.at<gtsam::Pose3>(X(step));
    for (uint i = start; i <= end; i++)
    {
        gtsam::Pose3 t = cp * cp.between(init_values.at<gtsam::Pose3>(X(i)));
        pcl::PointCloud<pcl::PointXYZSD> p;
        pcl::transformPointCloud(*points_cache[i], p, t.matrix());
        *smap += p;
    }

    // voxel_filter.setInputCloud(smap);
    // voxel_filter.filter(*smap);

    return smap;
}

void RadarPoseGraph::cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                             const nav_msgs::OdometryConstPtr odom)
{
    step++;

    pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_pc(new pcl::PointCloud<pcl::PointXYZSD>);
    fromROSMsg(*radar_msg, *radar_pc);
    voxel_filter.setInputCloud(radar_pc);
    voxel_filter.filter(*radar_pc);
    snr_filter.setInputCloud(radar_pc);
    snr_filter.filter(*radar_pc);
    points_cache.push_back(radar_pc);

    gtsam::Pose3 current_pose(gtsam::Rot3(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                          odom->pose.pose.orientation.y, odom->pose.pose.orientation.z),
                              gtsam::Point3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    gtsam::Pose3 tail_pose = init_values.at<gtsam::Pose3>(X(step - 1));
    tail_pose = tail_pose * last_pose.between(current_pose);
    path.poses.push_back(toROSPoseStamped(tail_pose, odom->header));
    init_values.insert(X(step), tail_pose);

    auto noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-1, 1e-1, 1e-1, 2e-2, 2e-2, 2e-2).finished());
    graph->add(gtsam::BetweenFactor<gtsam::Pose3>(X(step - 1), X(step), last_pose.between(current_pose), noise));

    // loop detect
    uint detect_point = (step > key_frame_search) ? step - key_frame_search : 0;
    if (detect_point - last_loop_detect > loop_close_rate)
    {
        auto l = detect_loop(init_values, detect_point);
        if (l.second != -1)
        {
            last_loop_detect = detect_point;

            // build submap
            pcl::PointCloud<pcl::PointXYZSD>::Ptr local_map =
                build_submap(l.first, l.first - key_frame_search, l.first + key_frame_search);
            pcl::PointCloud<pcl::PointXYZSD>::Ptr sub_map =
                build_submap(l.second, l.second - key_frame_search, l.second + key_frame_search);

            // find transform
            pcl::PointCloud<pcl::PointXYZSD> aligned_map;
            ndt_omp.setInputSource(local_map);
            ndt_omp.setInputTarget(sub_map);
            ndt_omp.align(aligned_map);
            Eigen::Matrix4f tf = ndt_omp.getFinalTransformation();

            if (ndt_omp.getFitnessScore() < 1)
            {
                ROS_INFO("loop detected, score %.4f", ndt_omp.getFitnessScore());
                loop_cache.push_back(l);

                // loop close on 2d only
                gtsam::Pose3 t(tf.cast<double>());
                gtsam::Pose3 tf3(gtsam::Rot3::Yaw(t.rotation().yaw()),
                                 gtsam::Point3(t.translation().x(), t.translation().y(), 0));
                gtsam::Pose3 correct_tf = tf3 * init_values.at<gtsam::Pose3>(X(l.first));
                gtsam::Pose3 between_tf = correct_tf.between(init_values.at<gtsam::Pose3>(X(l.second)));
                graph->add(gtsam::BetweenFactor<gtsam::Pose3>(X(l.first), X(l.second),
                                                              between_tf, loop_noise_model));

                // optimize
                gtsam::LevenbergMarquardtOptimizer optimizer(*graph, init_values);
                init_values = optimizer.optimize();

                // update visualization
                for (uint i = 0; i < step; i++)
                {
                    path.poses[i].pose = toROSPose(init_values.at<gtsam::Pose3>(X(i + 1)));
                }

                loop_marker.points.clear();
                for (auto p : loop_cache)
                {
                    gtsam::Point3 p1 = init_values.at<gtsam::Pose3>(X(p.first)).translation();
                    gtsam::Point3 p2 = init_values.at<gtsam::Pose3>(X(p.second)).translation();
                    loop_marker.points.push_back(toROSPoint(p1));
                    loop_marker.points.push_back(toROSPoint(p2));
                }
                pub_loop_marker.publish(loop_marker);
            }
            else
            {
                ROS_INFO("loop denied, score %.4f", ndt_omp.getFitnessScore());
            }

            // points visualization
            sensor_msgs::PointCloud2 msg;
            toROSMsg(*local_map, msg);
            pub_local.publish(msg);
            toROSMsg(*sub_map, msg);
            pub_subm.publish(msg);
            toROSMsg(aligned_map, msg);
            pub_aligned.publish(msg);
        }
    }

    // visualize poses
    path.header.stamp = odom->header.stamp;
    pub_path.publish(path);

    // broadcast transformation
    tf::Transform transform;
    auto trans = tail_pose.translation();
    auto orien = tail_pose.rotation().toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    br.sendTransform(tf::StampedTransform(transform, odom->header.stamp, "map", "base"));

    // reset
    last_pose = current_pose;
}

RadarPoseGraph::~RadarPoseGraph()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_pose_graph");
    ros::NodeHandle nh;
    RadarPoseGraph rpg(&nh);
    ros::spin();
    return 0;
}