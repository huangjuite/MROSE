
// std includes
#include <memory>

// ROS includes
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// RIO includes
#include "manager/DIOManager.hpp"
#include "core/RadarPointType.hpp"
#include "radar/RadarVelocityEstimatorEigen.hpp"
#include "core/params.hpp"
#include "core/rplrio_types.hpp"

template <class PointType> class DIORosNode : public rclcpp::Node {
private:
  typedef RadarVelocityEstimatorEigen<PointType> VestType;
  typedef typename pcl::PointCloud<PointType>::ConstPtr PclConstPtr;
  typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
  typedef typename pcl::PointCloud<PointType> Pcl;

  std::string imu_frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      sub_radars;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr>
      sub_vel;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      pub_twist;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_bias, pub_err;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radar;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  nav_msgs::msg::Path path_msg;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;

  void publish_all(
      const std::shared_ptr<rplrio::Keyframe<PointType>> keyframe);

public:
  DIORosNode();
  ~DIORosNode();
  rplrio::Params params;
  std::unique_ptr<rplrio::DIOManager<PointType>> dio_manager;
  void cb_radar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
  void cb_vel(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr vel_msg);
  void cb_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu);
};
