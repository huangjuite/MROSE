
// std includes
#include <memory>

// ROS includes
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include "DIOManager.h"
#include "RadarPointType.h"
#include "RadarVelocityEstimatorCeres.h"
#include "params.h"
#include "rplrio_types.h"

typedef pcl::PointXYZSD PointType;
typedef RadarVelocityEstimatorCeres VestType;

class DIORosNode : public rclcpp::Node {
private:
  rplrio::Params params;
  std::unique_ptr<rplrio::DIOManager<PointType>> dio_manager;

  // ROS2: NodeHandle is not needed, use 'this' as Node pointer

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      sub_radars;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_v;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      pub_twist;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_bias;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_error;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_radar;

  std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  std::shared_ptr<tf2_ros::TransformListener> listener;

  void cb_radar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);
  void cb_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void publish_all(
      const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe);

public:
  DIORosNode();
  ~DIORosNode();
};
