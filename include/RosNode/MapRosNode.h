
// std includes
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>

// ROS includes
#include <geometry_msgs/msg/pose_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

// PCL includes
#define PCL_NO_PRECOMPILE
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "params.h"

typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
                                                  nav_msgs::msg::Odometry>
    MySyncPolicy;

template <class PointT> class MapOptimize : public rclcpp::Node {
private:
  typedef typename pcl::PointCloud<PointT>::Ptr PclPtr;

  rplrio::Params params;

  std::vector<PclPtr> points_cache;
  gtsam::Pose3 last_pose;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;

  nav_msgs::msg::Path path;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_radar;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

  PclPtr global_map;
  std::mutex global_map_mtx;
  pcl::VoxelGrid<PointT> voxel_filter;

  void cb_node(const sensor_msgs::msg::PointCloud2::ConstSharedPtr radar_msg,
               const nav_msgs::msg::Odometry::ConstSharedPtr odom);
public:
  MapOptimize();
  ~MapOptimize();

  void render_map();
};
