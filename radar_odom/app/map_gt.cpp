// std includes
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <radar_odom/extrinsicConfig.h>
#include <dynamic_reconfigure/server.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

#include "RadarVelocityEstimatorCeres.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

pcl::PointCloud<pcl::PointXYZSD> fullmap;
pcl::VoxelGrid<pcl::PointXYZSD> voxel_filter;
std::vector<gtsam::Pose3> odom_gt;
std::vector<double> stamp_gt;
RadarVelocityEstimatorCeres vest;
ros::Publisher pub_path, pub_map;
nav_msgs::Path path;
double rotate = M_PI;

std::vector<gtsam::Pose3> odom_csv(std::string path)
{
    std::vector<gtsam::Pose3> gt;

    std::ifstream file;
    file.open(path, std::ios::in);

    std::string line;
    std::getline(file, line, '\n');

    // gtsam::Pose3 tf(gtsam::Rot3::Yaw(M_PI / 2), gtsam::Point3(0, 0, 0));
    gtsam::Pose3 tf(gtsam::Rot3::Yaw(rotate), gtsam::Point3(0, 0, 0));

    while (file.good())
    {
        std::string line;
        std::getline(file, line, '\n');

        if (line.empty())
            continue;

        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream ss(line);

        double stamp;
        ss >> stamp;
        uint seq;
        ss >> seq;

        std::vector<double> v;
        std::copy(std::istream_iterator<double>(ss),
                  std::istream_iterator<double>(),
                  std::back_inserter(v));

        // tx, ty, tz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az

        gt.emplace_back(tf * gtsam::Pose3(gtsam::Rot3(v[6], v[3], v[4], v[5]),
                                          gtsam::Point3(v[0], v[1], v[2])));
        stamp_gt.emplace_back(stamp);
    }
    file.close();

    return gt;
}

void pub_once()
{
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    std::cout << odom_gt.size() << std::endl;

    for (uint i = 0; i < odom_gt.size(); i++)
    {
        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();
        path.poses.emplace_back(toROSPoseStamped(odom_gt[i], header));
    }

    pub_path.publish(path);
}

bool cb_publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{

    pub_once();

    rep.success = true;
    return true;
}

bool cb_pubmap(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{

    std::cout << fullmap.size() << std::endl;
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(fullmap, map_msg);
    map_msg.header.frame_id = "map";
    pub_map.publish(map_msg);

    rep.success = true;
    return true;
}

void build_map(double t, pcl::PointCloud<pcl::PointXYZSD> pc)
{

    auto it = std::lower_bound(stamp_gt.begin(), stamp_gt.end(), t);
    if (it != stamp_gt.end() && it != stamp_gt.begin())
    {
        std::cout << t << ", " << *it << std::endl;

        double ratio = abs(t - *it) / *(it + 1);
        size_t idx = it - stamp_gt.begin();

        gtsam::Pose3 p1 = odom_gt[idx], p2 = odom_gt[idx + 1];
        gtsam::Pose3 p = p1.interpolateRt(p2, ratio);

        pcl::transformPointCloud(pc, pc, p.matrix());

        voxel_filter.setInputCloud(pc.makeShared());
        voxel_filter.filter(pc);
        fullmap += pc;
    }
}

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    ros::init(argc, argv, "map_gt");
    ros::NodeHandle nh;

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    ros::param::get("~rotate", rotate);
    odom_gt = odom_csv(csv_path);

    pub_path = nh.advertise<nav_msgs::Path>("gt_path", 1);
    pub_map = nh.advertise<sensor_msgs::PointCloud2>("radar/map", 1);
    auto pub_srv = nh.advertiseService("pub_gt", cb_publish);
    auto map_srv = nh.advertiseService("pub_map", cb_pubmap);

    voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    rplrio::Params params = rplrio::get_params(nh);
    vest.set_inlier_thresh(0.124);

    pub_once();

    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    std::vector<double> stamps;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());
    // topics.push_back("/velodyne_points");
    topics.push_back("/radar0/points");
    topics.push_back("/radar1/points");

    for (auto b : bags)
    {
        rosbag::Bag bag;
        bag.open(b, rosbag::bagmode::Read);
        std::cout << b << std::endl;

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {

            sensor_msgs::PointCloud2::ConstPtr point_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (point_msg != NULL)
            {
                std::string rname = point_msg->header.frame_id;
                auto R = params.radars[rname].R_r_b;
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
                T.block(0, 0, 3, 3) = R;

                pcl::PointCloud<pcl::PointXYZSD>::Ptr pc(new pcl::PointCloud<pcl::PointXYZSD>);

                pcl::fromROSMsg(*point_msg, *pc);
                auto v = vest.optimize(pc);
                vest.get_inlier(pc, v);

                pcl::transformPointCloud(*pc, *pc, T);
                build_map(point_msg->header.stamp.toSec(), *pc);
                continue;
            }
        }

        bag.close();
    }

    voxel_filter.setInputCloud(fullmap.makeShared());
    voxel_filter.filter(fullmap);
    std::cout << fullmap.size() << std::endl;

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(fullmap, map_msg);
    map_msg.header.frame_id = "map";
    pub_map.publish(map_msg);

    ros::spin();
    return 0;
}