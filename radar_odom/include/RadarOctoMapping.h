
// std includes
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include <radar_odom/mappingConfig.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

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

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include "utils.h"

using namespace ros;
using namespace pcl;
using namespace gtsam;
using namespace std;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Path> MySyncPolicy;
typedef octomap::OcTree OcTreeT;

class RadarOctoMapping
{
private:
    bool use_voxel = false;
    float process_voxel_size = 0.2;
    float map_voxel_size = 0;
    float snr_thresh = 50;
    map<uint, pcl::PointCloud<PointXYZIS>::Ptr> cache;
    pcl::PointCloud<PointXYZIS>::Ptr full_map;
    Pose3 last_pose;

    Publisher pub_map, m_mapPub, m_markerPub;
    Subscriber sub_loop;
    nav_msgs::Path full_path;
    visualization_msgs::Marker loop_marker;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_radar;
    message_filters::Subscriber<nav_msgs::Path> sub_path;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    dynamic_reconfigure::Server<radar_odom::mappingConfig> server;
    dynamic_reconfigure::Server<radar_odom::mappingConfig>::CallbackType f;

    VoxelGrid<PointXYZIS> voxel_filter;
    VoxelGrid<PointXYZIS> map_voxel_filter;
    PassThrough<PointXYZIS> intensity_filter;
    PassThrough<PointXYZIS> x_filter;
    PassThrough<PointXYZIS> y_filter;

    OcTreeT *m_octree;
    octomap::KeyRay m_keyRay; // temp storage for ray casting
    octomap::OcTreeKey m_updateBBXMin;
    octomap::OcTreeKey m_updateBBXMax;
    double m_res;
    unsigned m_treeDepth;
    unsigned m_maxTreeDepth;
    double m_colorFactor;
    double m_minSizeX;
    double m_minSizeY;
    double m_occupancyMinZ;
    double m_occupancyMaxZ;

    // downprojected 2D map:
    bool m_incrementalUpdate = false;
    nav_msgs::OccupancyGrid m_gridmap;
    octomap::OcTreeKey m_paddedMinKey;
    unsigned m_multires2DScale;
    bool m_projectCompleteMap;

    void cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                 const nav_msgs::PathConstPtr path);
    void update_map();
    void cb_config(radar_odom::mappingConfig &config, uint32_t level);
    void cb_loop(const visualization_msgs::Marker loop);
    void insert_scan(const pcl::PointCloud<pcl::PointXYZIS>::ConstPtr map_part,
                     const octomath::Vector3 sensorOrigin);
    void reset_octree_map();
    void handlePreNodeTraversal(const ros::Time &rostime);
    void handlePostNodeTraversal(const ros::Time &rostime);
    void handleOccupiedNode(const OcTreeT::iterator &it);
    void handleFreeNode(const OcTreeT::iterator &it);
    void handleOccupiedNodeInBBX(const OcTreeT::iterator &it);
    void handleFreeNodeInBBX(const OcTreeT::iterator &it);
    void update2DMap(const OcTreeT::iterator &it, bool occupied);
    void publishProjected2DMap(const ros::Time &rostime);
    void adjustMapData(nav_msgs::OccupancyGrid &map, const nav_msgs::MapMetaData &oldMapInfo) const;

    visualization_msgs::MarkerArray getROSMarkerArray(ros::Time stamp);
    std_msgs::ColorRGBA heightMapColor(double h);

protected:
    inline static void updateMinKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &min)
    {
        for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
    };

    inline static void updateMaxKey(const octomap::OcTreeKey &in, octomap::OcTreeKey &max)
    {
        for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
    };

    /// Test if key is within update area of map (2D, ignores height)
    inline bool isInUpdateBBX(const OcTreeT::iterator &it) const
    {
        // 2^(tree_depth-depth) voxels wide:
        unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
        octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
        return (key[0] + voxelWidth >= m_updateBBXMin[0] && key[1] + voxelWidth >= m_updateBBXMin[1] && key[0] <= m_updateBBXMax[0] && key[1] <= m_updateBBXMax[1]);
    };

    inline unsigned mapIdx(int i, int j) const
    {
        return m_gridmap.info.width * j + i;
    };

    inline unsigned mapIdx(const octomap::OcTreeKey &key) const
    {
        return mapIdx((key[0] - m_paddedMinKey[0]) / m_multires2DScale,
                      (key[1] - m_paddedMinKey[1]) / m_multires2DScale);
    };

    inline bool mapChanged(const nav_msgs::MapMetaData &oldMapInfo, const nav_msgs::MapMetaData &newMapInfo)
    {
        return (oldMapInfo.height != newMapInfo.height || oldMapInfo.width != newMapInfo.width || oldMapInfo.origin.position.x != newMapInfo.origin.position.x || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
    };

public:
    RadarOctoMapping(NodeHandle *nh);
    ~RadarOctoMapping();
};
