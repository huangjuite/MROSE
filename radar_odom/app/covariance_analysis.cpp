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

#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "VelocityPoseExtrinsicFactor.h"
#include "AltitudeFactor.h"

#include "RadarVelocityEstimatorCeresHCS.h"
#include "RadarVelocityEstimatorCeres.h"
#include "StaticInitializer.hpp"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::E;
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

std::vector<rplrio::Odom> odom_csv(std::string path)
{
    std::vector<rplrio::Odom> gt;

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

        gt.emplace_back(rplrio::Odom(stamp, seq, pose,
                                     Eigen::Vector3d(v[7], v[8], v[9]),
                                     Eigen::Vector3d(v[10], v[11], v[12])));
    }

    return gt;
}

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);

    ros::init(argc, argv, "cov_analysis");
    ros::NodeHandle nh;
    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);

    std::string csv_path;
    ros::param::get("~gt_path", csv_path);
    auto odom_vio = odom_csv(csv_path);

    rplrio::Params params = rplrio::get_params(nh);
    std::vector<double> stamps;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());

    auto radarParam = params.radars[params.radar_list[0]];
    topics.push_back(radarParam.topic);

    RadarVelocityEstimatorCeres v_opt_cartesian;
    RadarVelocityEstimatorCeresHCS v_opt_horizontal;
    auto radar_p = params.radars[params.radar_list[0]];
    v_opt_horizontal.set_uncertainty(
        radar_p.doppler_std, radar_p.azimuth_std, radar_p.elevation_std);

    std::ofstream log_file;
    log_file.open("/tmp/cov_tmp.csv");
    log_file << std::fixed << std::setprecision(15);
    log_file << "stamp, "
             << "vx_cart, vy_cart, vz_cart, "
             << "covx_cart, covy_cart, covz_cart, "
             << "vx_hori, vy_hori, vz_hori, "
             << "covx_hori, covy_hori, covz_hori, "
             << "vx_vio, vy_vio, vz_vio" << std::endl;

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
                pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_pc;
                radar_pc.reset(new pcl::PointCloud<pcl::PointXYZSD>);
                pcl::fromROSMsg(*radar_msg, *radar_pc);
                double current_t = radar_msg->header.stamp.toSec();

                std::cout << "\033c";
                std::cout << current_t << ", " << std::endl;

                auto it = std::lower_bound(odom_vio.begin(), odom_vio.end(), current_t);
                if (it == odom_vio.begin() || it == odom_vio.end())
                    continue;
                double ratio = (current_t - (it - 1)->stamp) / (it->stamp - (it - 1)->stamp);
                auto body_v = ratio * it->v + (1 - ratio) * (it - 1)->v;

                v_opt_cartesian.set_initial_velocity(Eigen::Vector3d(1, 0, 0));
                gtsam::Vector3 measured_v_cartesian = v_opt_cartesian.optimize(radar_pc);
                gtsam::Matrix3 cov_cart = v_opt_cartesian.get_covariance(
                    radar_pc, measured_v_cartesian);
                measured_v_cartesian = radarParam.R_r_b * measured_v_cartesian;

                v_opt_horizontal.set_initial_velocity(Eigen::Vector3d(1, 0, 0));
                gtsam::Vector3 measured_v_horizontal = v_opt_horizontal.optimize(radar_pc);
                gtsam::Matrix3 cov_horizontal = v_opt_horizontal.get_covariance(
                    radar_pc, measured_v_horizontal);
                measured_v_horizontal = radarParam.R_r_b * measured_v_horizontal;

                log_file << current_t << ", ";
                log_file << measured_v_cartesian[0] << ", "
                         << measured_v_cartesian[1] << ", "
                         << measured_v_cartesian[2] << ", "
                         << cov_cart(0, 0) << ", "
                         << cov_cart(1, 1) << ", "
                         << cov_cart(2, 2) << ", ";
                log_file << measured_v_horizontal[0] << ", "
                         << measured_v_horizontal[1] << ", "
                         << measured_v_horizontal[2] << ", "
                         << cov_horizontal(0, 0) << ", "
                         << cov_horizontal(1, 1) << ", "
                         << cov_horizontal(2, 2) << ", ";
                log_file << body_v[0] << ", "
                         << body_v[1] << ", "
                         << body_v[2] << std::endl;
            }
        }

        bag.close();
    }

    log_file.close();

    return 0;
}