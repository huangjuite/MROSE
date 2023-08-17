
// std includes
#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL includes
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pclomp/ndt_omp.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

#include "utils.h"
#include "params.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;


template <class PointT>
class MapOptimize
{
private:
    typedef typename pcl::PointCloud<PointT>::Ptr PclPtr;

    rplrio::Params params;
    uint step = 0;
    uint last_loop_detect = 0;

    std::vector<PclPtr> points_cache;
    std::vector<std::pair<int, int>> loop_cache;
    gtsam::Pose3 last_pose;

    ros::Publisher pub_local, pub_subm, pub_aligned, pub_path, pub_loop_marker;
    ros::Publisher pub_map;
    nav_msgs::Path path;
    visualization_msgs::Marker loop_marker;

    pcl::VoxelGrid<PointT> voxel_filter;
    pcl::PassThrough<PointT> snr_filter;
    pcl::VoxelGrid<PointT> map_voxel_filter;
    pcl::PassThrough<PointT> map_snr_filter;
    pclomp::NormalDistributionsTransform<PointT, PointT> ndt_omp;

    tf::TransformBroadcaster br;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_radar;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
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
    void cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                 const nav_msgs::OdometryConstPtr odom);
    void publishMap();

public:
    MapOptimize(ros::NodeHandle *nh);
    ~MapOptimize();

    void render_map();
};
