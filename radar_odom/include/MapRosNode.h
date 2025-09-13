
// std includes
#include <memory>
#include <mutex>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <vector>

// ROS includes
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

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

// GICP includes
#include "small_gicp/pcl/pcl_registration.hpp"

// GTSAM includes.
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include "params.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,
                                                  nav_msgs::msg::Odometry>
    MySyncPolicy;

template <class PointT> class MapOptimize : public rclcpp::Node {
private:
  typedef typename pcl::PointCloud<PointT>::Ptr PclPtr;

  rplrio::Params params;
  uint step = 0;
  uint last_loop_detect = 0;

  std::vector<PclPtr> points_cache;
  std::vector<std::pair<int, int>> loop_cache;
  gtsam::Pose3 last_pose;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local,
      pub_subm, pub_aligned, pub_map;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_loop_marker;

  nav_msgs::msg::Path path;
  visualization_msgs::msg::Marker loop_marker;

  pcl::VoxelGrid<PointT> voxel_filter;
  pcl::PassThrough<PointT> snr_filter;
  pcl::VoxelGrid<PointT> map_voxel_filter;
  pcl::PassThrough<PointT> map_snr_filter;
  small_gicp::RegistrationPCL<PointT, PointT> gicp_reg;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_radar;
  message_filters::Subscriber<nav_msgs::msg::Odometry> sub_odom;
  std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

  gtsam::Values init_values;
  gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model;
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise_model;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;
  // std::unique_ptr<gtsam::ISAM2> isam2;

  bool a_loop_closed = false;
  pcl::PointCloud<PointT> global_map;
  std::mutex global_map_mtx;

  std::pair<int, int> detect_loop(gtsam::Values &nodes, uint step);
  PclPtr build_submap(uint step, uint start, uint end);
  void cb_node(const sensor_msgs::msg::PointCloud2::ConstSharedPtr radar_msg,
               const nav_msgs::msg::Odometry::ConstSharedPtr odom);
  void publishMap();

public:
  MapOptimize();
  ~MapOptimize();

  void render_map();
};
