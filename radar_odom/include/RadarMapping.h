
// std includes
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include <radar_odom/mappingConfig.h>

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
#include <pclradar/RadarPointType.h>

#include "utils.h"

using namespace ros;
using namespace pcl;
using namespace gtsam;
using namespace std;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Path> MySyncPolicy;

class RadarMapping
{
private:
    bool use_voxel = false;
    float process_voxel_size = 0.2;
    float map_voxel_size = 0;
    float snr_thresh = 50;
    map<uint, pcl::PointCloud<PointXYZSD>::Ptr> cache;
    pcl::PointCloud<PointXYZSD>::Ptr full_map;
    Pose3 last_pose;

    Publisher pub_map;
    Subscriber sub_loop;
    nav_msgs::Path full_path;
    visualization_msgs::Marker loop_marker;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_radar;
    message_filters::Subscriber<nav_msgs::Path> sub_path;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    dynamic_reconfigure::Server<radar_odom::mappingConfig> server;
    dynamic_reconfigure::Server<radar_odom::mappingConfig>::CallbackType f;

    VoxelGrid<PointXYZSD> voxel_filter;
    VoxelGrid<PointXYZSD> map_voxel_filter;
    PassThrough<PointXYZSD> snr_filter;
    PassThrough<PointXYZSD> x_filter;
    PassThrough<PointXYZSD> y_filter;

    void cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                 const nav_msgs::PathConstPtr path);
    void update_map();
    void cb_config(radar_odom::mappingConfig &config, uint32_t level);
    void cb_loop(const visualization_msgs::Marker loop);

public:
    RadarMapping(NodeHandle *nh);
    ~RadarMapping();
};
