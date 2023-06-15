
// std includes
#include <string>
#include <vector>

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
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include "utils.h"

using namespace ros;
using namespace pcl;
using namespace gtsam;
using namespace std;

using symbol_shorthand::X; // Pose2 (x,y,r)

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

class RadarPoseGraph2d
{
private:
    int loop_close_rate = 50;  // step
    int loop_ignore = 50;      // step
    int key_frame_search = 20; // step
    double loop_distance_thresh = 0.5;
    bool enable_loop_close = true;
    float process_voxel_size = 0.2;
    float ndt_voxel_size = 1;
    float snr_thresh = 4;
    float max_range = 15;

    uint step = 0;
    uint last_loop_detect = 0;
    vector<pcl::PointCloud<PointXYZIS>::Ptr> points_cache;
    vector<pair<int, int>> loop_cache;
    Pose2 last_pose;

    Publisher pub_local, pub_subm, pub_aligned;
    Publisher pub_path;
    Publisher pub_loop_marker;
    nav_msgs::Path path;
    visualization_msgs::Marker loop_marker;

    VoxelGrid<PointXYZIS> voxel_filter;
    PassThrough<PointXYZIS> intensity_filter;
    PassThrough<PointXYZIS> x_filter;
    PassThrough<PointXYZIS> y_filter;
    pclomp::NormalDistributionsTransform<PointXYZIS, PointXYZIS> ndt_omp;

    tf::TransformBroadcaster br;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_radar;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    Values init_values;
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Robust::shared_ptr loop_noise_model;
    NonlinearFactorGraph *graph;
    ISAM2 *isam2;

    pair<int, int> detect_loop(Values &nodes, uint step);
    PointCloud<PointXYZIS>::Ptr build_submap(uint origin_frame, uint start, uint end);
    void cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                 const nav_msgs::OdometryConstPtr odom);

public:
    RadarPoseGraph2d(NodeHandle *nh);
    ~RadarPoseGraph2d();
};
