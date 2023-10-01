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
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

#include "VelocityPoseMeasurementFactor.h"

#include "RadarVelocityEstimatorCeres.h"
#include "StaticInitializer.hpp"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

std::vector<rplrio::Odom> odom_vio;
ros::Publisher pub_path;
nav_msgs::Path path;
double rotate = M_PI;
std::vector<rplrio::Odom> odom_csv(std::string path)
{
    std::vector<rplrio::Odom> gt;

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

        gtsam::Pose3 pose = tf * gtsam::Pose3(gtsam::Rot3(v[6], v[3], v[4], v[5]),
                                              gtsam::Point3(v[0], v[1], v[2]));

        gt.emplace_back(rplrio::Odom(stamp, seq, pose,
                                     Eigen::Vector3d(v[7], v[8], v[9]),
                                     Eigen::Vector3d(v[10], v[11], v[12])));
    }
    file.close();

    return gt;
}

void pub_once()
{
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    std::cout << odom_vio.size() << std::endl;

    for (uint i = 0; i < odom_vio.size(); i++)
    {
        std_msgs::Header header;
        header.frame_id = "map";
        header.stamp = ros::Time::now();
        path.poses.emplace_back(toROSPoseStamped(odom_vio[i].pose, header));
    }

    pub_path.publish(path);
}

bool cb_publish(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &rep)
{

    pub_once();

    rep.success = true;
    return true;
}

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    ros::init(argc, argv, "gt_path");
    ros::NodeHandle nh;

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    ros::param::get("~rotate", rotate);
    odom_vio = odom_csv(csv_path);

    pub_path = nh.advertise<nav_msgs::Path>("gt_path", 1);

    auto pub_srv = nh.advertiseService("pub_gt", cb_publish);

    pub_once();

    ros::spin();
    return 0;
}