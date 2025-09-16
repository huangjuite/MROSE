#ifndef UTILS_H
#define UTILS_H

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

// GTSAM includes.
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "RadarPointType.h"
#include "params.h"
#include <Eigen/Dense>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <string>

namespace rplrio {

visualization_msgs::msg::Marker velocityMarker(
    gtsam::Vector3 velocity, gtsam::Vector3 position = gtsam::Vector3(0, 0, 0),
    std::string frame_id = "radar", std::string ns = "radar_doppler");

std::array<double, 36> toROSCov(Eigen::MatrixXd cov);

Eigen::MatrixXd fromROSCov(std::array<double, 36> ros_cov);

Eigen::MatrixXd fromROSCov_2d(std::array<double, 36> ros_cov);

nav_msgs::msg::Odometry toOdometry(Eigen::Matrix4d global_pose,
                                   gtsam::Matrix covariance = gtsam::Matrix());

geometry_msgs::msg::TwistWithCovarianceStamped
toROSTwist(gtsam::Vector3 v, gtsam::Matrix cov, std_msgs::msg::Header header);

geometry_msgs::msg::Twist toROStwist(gtsam::imuBias::ConstantBias bias);

gtsam::Pose3 toPose3(const gtsam::Pose2 gtsam_pose);

gtsam::Pose3 toPose3(const nav_msgs::msg::Odometry odom);

gtsam::Pose3 toPose3(const geometry_msgs::msg::PoseStamped pst);

geometry_msgs::msg::Point toROSPoint(gtsam::Point3 gtsam_point);

geometry_msgs::msg::Point toROSPoint(gtsam::Point2 gtsam_point);

geometry_msgs::msg::Pose toROSPose(gtsam::Pose3 gtsam_pose);

geometry_msgs::msg::Pose toROSPose(gtsam::Pose2 gtsam_pose);

geometry_msgs::msg::PoseStamped toROSPoseStamped(const gtsam::Pose3 gtsam_pose,
                                                 std_msgs::msg::Header header);

geometry_msgs::msg::PoseStamped toROSPoseStamped(const gtsam::Pose2 gtsam_pose,
                                                 std_msgs::msg::Header header);

geometry_msgs::msg::TransformStamped
toROSTransform(const gtsam::Pose3 gtsam_pose, std::string frame_id,
               std::string child_id, std_msgs::msg::Header::_stamp_type stamp);

pcl::PointCloud<pcl::PointXYZSD>
from_xrio_type(pcl::PointCloud<pcl::PointXYZSNV> &radar_xrio);

void logdB_SNR(pcl::PointCloud<pcl::PointXYZSD>::Ptr pc);

void logdB_SNR(pcl::PointCloud<pcl::PointXYZSDW>::Ptr pc);

void logdB_SNR(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);

Params get_params(std::string cfg_path);

builtin_interfaces::msg::Time create_stamp_from_seconds(double seconds);

} // namespace rplrio

#endif
