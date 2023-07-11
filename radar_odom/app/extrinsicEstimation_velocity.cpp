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
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "VelocityPoseExtrinsicFactor.h"
#include "AltitudeFactor.h"

#include "RadarVelocityEstimatorCeresHCS.h"
#include "RadarVelocityEstimatorCeres.h"
#include "StaticInitializer.hpp"

#include "types.h"
#include "utils.h"
#include "params.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

std::vector<rio::Odom> odom_csv(std::string path)
{
    std::vector<rio::Odom> gt;

    std::ifstream file;
    file.open(path, std::ios::in);

    std::string line;
    std::getline(file, line, '\n');

    gtsam::Pose3 tf(gtsam::Rot3::Yaw(M_PI / 2), gtsam::Point3(0, 0, 0));

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

        gt.emplace_back(rio::Odom(stamp, seq, pose,
                                  Eigen::Vector3d(v[7], v[8], v[9]),
                                  Eigen::Vector3d(v[10], v[11], v[12])));
    }

    return gt;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_extrinsic");
    ros::NodeHandle nh;
    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    auto odom_vio = odom_csv(csv_path);

    rplrio::Params params = rplrio::get_params(nh);

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());
    topics.push_back(params.radar_topic);

    RadarVelocityEstimatorCeres v_opt;
    std::cout.precision(20);
    std::ofstream odom_file;
    odom_file.open("/tmp/odom_tmp.csv");
    odom_file << std::fixed << std::setprecision(15);
    // odom_file << "stamp, vx, vy, vz, rx, ry, rz" << std::endl;

    for (auto b : bags)
    {
        rosbag::Bag bag;
        bag.open(b, rosbag::bagmode::Read);

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr radar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (radar_msg != NULL)
            {
                pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
                radar_pc.reset(new pcl::PointCloud<pcl::PointXYZIS>);
                pcl::fromROSMsg(*radar_msg, *radar_pc);
                double current_t = radar_msg->header.stamp.toSec();

                auto it = std::lower_bound(odom_vio.begin(), odom_vio.end(), current_t);

                if (it == odom_vio.begin() || it == odom_vio.end())
                    continue;

                double ratio = (current_t - (it - 1)->stamp) / (it->stamp - (it - 1)->stamp);
                auto body_v = ratio * it->v + (1 - ratio) * (it - 1)->v;

                std::cout << current_t << ", " << it->stamp << std::endl;

                v_opt.set_initial_velocity(gtsam::Vector3::Identity());
                gtsam::Vector3 measured_v = v_opt.optimize(radar_pc);
                gtsam::Matrix3 radar_velocity_cov = v_opt.get_covariance(
                    radar_pc, measured_v);

                measured_v = params.R_r_b * measured_v;

                odom_file << current_t << ", ";
                odom_file << measured_v[0] << ", ";
                odom_file << measured_v[1] << ", ";
                odom_file << measured_v[2] << ", ";
                odom_file << body_v[0] << ", ";
                odom_file << body_v[1] << ", ";
                odom_file << body_v[2] << std::endl;
            }
        }

        odom_file.close();

        bag.close();
    }

    return 0;
}