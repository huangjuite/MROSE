#include "RadarPoseGraph2dFGR.h"

RadarPoseGraph2dFGR::RadarPoseGraph2dFGR(NodeHandle *nh)
{
    param::get("~loop_close_rate", loop_close_rate);
    param::get("~loop_ignore", loop_ignore);
    param::get("~key_frame_search", key_frame_search);
    param::get("~loop_distance_thresh", loop_distance_thresh);
    param::get("~enable_loop_close", enable_loop_close);
    param::get("~process_voxel_size", process_voxel_size);
    param::get("~snr_thresh", snr_thresh);
    param::get("~max_range", max_range);

    // pcl init
    points_cache.push_back(nullptr); // align index with graph
    voxel_filter.setLeafSize(process_voxel_size, process_voxel_size, process_voxel_size);
    intensity_filter.setFilterFieldName("intensity");
    intensity_filter.setFilterLimits(snr_thresh, FLT_MAX);

    // gtsam init
    graph = new NonlinearFactorGraph();
    Pose2 prior_pose(0, 0, 0);
    pose_noise_model = noiseModel::Diagonal::Sigmas(
        (Vector(3) << 1e-3, 1e-2, 1e-2).finished());

    loop_noise_model = noiseModel::Robust::Create(
        noiseModel::mEstimator::Huber::Create(1.345),
        noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.05)));

    graph->addPrior(X(0), prior_pose, pose_noise_model);
    init_values.insert(X(0), prior_pose);
    last_pose = prior_pose;

    // ros init
    loop_marker.header.frame_id = "map";
    loop_marker.action = visualization_msgs::Marker::ADD;
    loop_marker.ns = "loop_detected";
    loop_marker.id = 0;
    loop_marker.type = visualization_msgs::Marker::LINE_LIST;
    loop_marker.pose = toROSPose(Pose3::Identity());
    loop_marker.scale.x = 0.1;
    loop_marker.color.a = 1;
    loop_marker.color.g = 1;
    loop_marker.color.r = 1;
    path.header.frame_id = "map";

    fast_global_registration = nh->serviceClient<radar_odom::Registration>("global_registration");

    pub_local = nh->advertise<sensor_msgs::PointCloud2>("local_map", 1);
    pub_subm = nh->advertise<sensor_msgs::PointCloud2>("sub_map", 1);
    pub_aligned = nh->advertise<sensor_msgs::PointCloud2>("aligned_map", 1);

    pub_path = nh->advertise<nav_msgs::Path>("radar_path", 1);
    pub_loop_marker = nh->advertise<visualization_msgs::Marker>("loop_detected", 1);
    sub_radar.subscribe(*nh, "radar_inlier", 20);
    sub_odom.subscribe(*nh, "dopplerIO", 20);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), sub_radar, sub_odom);
    sync->registerCallback(boost::bind(&RadarPoseGraph2dFGR::cb_node, this, _1, _2));
    ROS_INFO("radar pose graph init");
}

pair<int, int> RadarPoseGraph2dFGR::detect_loop(Values &nodes, uint step)
{
    pair<int, int> close_pairs{step, -1};

    if (step < loop_ignore)
        return close_pairs;

    double min_dis = 1e4;
    Pose2 head_pose = nodes.at<Pose2>(X(step));

    for (uint i = step - loop_ignore; i > key_frame_search; i--)
    {
        Pose2 node_pose = nodes.at<Pose2>(X(i));
        double dis = distance2(node_pose.translation(), head_pose.translation());
        if (dis < loop_distance_thresh && dis < min_dis)
        {
            close_pairs.second = i;
            min_dis = dis;
        }
    }

    return close_pairs;
}

PointCloud<PointXYZIS>::Ptr RadarPoseGraph2dFGR::build_submap(uint origin_frame, uint start, uint end)
{
    PointCloud<PointXYZIS>::Ptr smap(new PointCloud<PointXYZIS>);
    smap->header.frame_id = "map";
    Pose2 cp = init_values.at<Pose2>(X(origin_frame));

    for (uint i = start; i <= end; i++)
    {
        Pose2 t2 = cp * cp.between(init_values.at<Pose2>(X(i)));
        Pose3 t = toPose3(t2);
        PointCloud<PointXYZIS> p;
        transformPointCloud(*points_cache[i], p, t.matrix());
        *smap += p;
    }

    voxel_filter.setInputCloud(smap);
    voxel_filter.filter(*smap);

    return smap;
}

void RadarPoseGraph2dFGR::cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                                  const nav_msgs::OdometryConstPtr odom)
{
    step++;

    PointCloud<PointXYZIS>::Ptr radar_pc(new PointCloud<PointXYZIS>);
    fromROSMsg(*radar_msg, *radar_pc);
    logdB_SNR(radar_pc);

    intensity_filter.setInputCloud(radar_pc);
    intensity_filter.filter(*radar_pc);
    points_cache.push_back(radar_pc);

    Pose3 cp3 = toPose3(*odom);
    Pose2 current_pose(cp3.x(), cp3.y(), cp3.rotation().yaw());
    Pose2 tail_pose = init_values.at<Pose2>(X(step - 1));
    tail_pose = tail_pose * last_pose.between(current_pose);
    path.poses.push_back(toROSPoseStamped(tail_pose, odom->header));
    init_values.insert(X(step), tail_pose);

    // auto noise = noiseModel::Gaussian::Covariance(fromROSCov_2d(odom->pose.covariance));
    // auto noise = noiseModel::Diagonal::Covariance(fromROSCov_2d(odom->pose.covariance));
    auto noise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph->add(BetweenFactor<Pose2>(X(step - 1), X(step), last_pose.between(current_pose), noise));

    // loop detect
    bool loop_is_closed = false;
    uint detect_point = (step > key_frame_search) ? step - key_frame_search : 0;

    if (enable_loop_close && detect_point - last_loop_detect > loop_close_rate)
    {
        last_loop_detect = detect_point;
        auto l = detect_loop(init_values, detect_point);
        if (l.second != -1)
        {
            // build submap
            PointCloud<PointXYZIS> aligned_map;
            PointCloud<PointXYZIS>::Ptr local_map =
                build_submap(l.first, l.first - key_frame_search, l.first + key_frame_search);
            PointCloud<PointXYZIS>::Ptr sub_map =
                build_submap(l.second, l.second - key_frame_search, l.second + key_frame_search);

            // find transform
            Pose2 pose_first = init_values.at<Pose2>(X(l.first));
            Pose2 pose_second = init_values.at<Pose2>(X(l.second));

            radar_odom::Registration fgr;
            toROSMsg(*local_map, fgr.request.local_map);
            toROSMsg(*sub_map, fgr.request.sub_map);

            if (fast_global_registration.call(fgr))
            {
                geometry_msgs::PoseStamped tf_pose;
                tf_pose.pose = fgr.response.tf;
                Pose3 tf3 = toPose3(tf_pose);
                transformPointCloud(*local_map, aligned_map, tf3.matrix());

                if (fgr.response.fitness > 0.8)
                {
                    ROS_INFO("loop detected, fitness %.4f", fgr.response.fitness);

                    Pose2 tf2(tf3.x(), tf3.y(), tf3.rotation().yaw());
                    Pose2 correct_tf2 = tf2 * pose_first;
                    Pose2 between_tf = correct_tf2.between(pose_second);
                    graph->add(BetweenFactor<Pose2>(X(l.first), X(l.second),
                                                    between_tf, loop_noise_model));

                    // optimize
                    LevenbergMarquardtOptimizer optimizer(*graph, init_values);
                    init_values = optimizer.optimize();

                    // update visualization
                    for (uint i = 0; i < step; i++)
                    {
                        path.poses[i].pose = toROSPose(init_values.at<Pose2>(X(i + 1)));
                    }

                    loop_cache.push_back(l);
                    loop_marker.points.clear();
                    for (auto p : loop_cache)
                    {
                        Point2 p1 = init_values.at<Pose2>(X(p.first)).translation();
                        Point2 p2 = init_values.at<Pose2>(X(p.second)).translation();
                        loop_marker.points.push_back(toROSPoint(p1));
                        loop_marker.points.push_back(toROSPoint(p2));
                    }
                    loop_is_closed = true;
                }
                else
                {
                    ROS_INFO("loop denied, fitness %.4f", fgr.response.fitness);
                }

                // points visualization
                pub_local.publish(fgr.request.local_map);
                pub_subm.publish(fgr.request.sub_map);
                sensor_msgs::PointCloud2 msg;
                toROSMsg(aligned_map, msg);
                pub_aligned.publish(msg);
            }
            else
            {
                ROS_WARN("GR service fail");
            }
            cout << "========================" << endl;
        }
    }

    // visualize poses
    path.header.stamp = odom->header.stamp;
    pub_path.publish(path);

    if (loop_is_closed)
        pub_loop_marker.publish(loop_marker);

    // broadcast transformation
    tf::Transform transform;
    auto trans = Point3(tail_pose.x(), tail_pose.y(), 0);
    auto orien = Rot3::Yaw(tail_pose.theta()).toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    br.sendTransform(tf::StampedTransform(transform, odom->header.stamp, "map", "base"));

    // reset
    last_pose = current_pose;
}

RadarPoseGraph2dFGR::~RadarPoseGraph2dFGR()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "radar_pose_graph");
    NodeHandle nh;
    RadarPoseGraph2dFGR rpg(&nh);
    spin();
    return 0;
}