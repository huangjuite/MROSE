#include <string>
#include <vector>
#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

// #include <pcl/keypoints/harris_3d.h>
// #include <pcl/keypoints/narf_keypoint.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include "LandmarkTracker.h"
#include "utils.h"

using namespace ros;
using namespace std;
using namespace pcl;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

class Clustering
{
private:
    NodeHandle *nh;
    Publisher pub_segment;
    Publisher pub_landmark;

    LandmarkTracker tracker;

    vector<vector<int>> colors = {
        {120, 28, 129}, {64, 67, 153}, {72, 139, 194}, {107, 178, 140}, {159, 190, 87}, {210, 179, 63}, {231, 126, 49}, {217, 33, 32}};

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_radar;
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    void cb_radar(const sensor_msgs::PointCloud2ConstPtr pc_msg,
                  const nav_msgs::OdometryConstPtr odom);
    std_msgs::ColorRGBA id_to_color(uint id);

public:
    Clustering(NodeHandle *nh);
    ~Clustering();
};

Clustering::Clustering(NodeHandle *nh)
{

    pub_segment = nh->advertise<sensor_msgs::PointCloud2>("radar_segment", 1);
    pub_landmark = nh->advertise<visualization_msgs::Marker>("radar_landmark", 1);

    sub_radar.subscribe(*nh, "radar_inlier", 20);
    sub_odom.subscribe(*nh, "dopplerIO", 20);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), sub_radar, sub_odom);
    sync->registerCallback(boost::bind(&Clustering::cb_radar, this, _1, _2));
}

void Clustering::cb_radar(const sensor_msgs::PointCloud2ConstPtr pc_msg,
                          const nav_msgs::OdometryConstPtr odom)
{
    PointCloud<PointXYZI>::Ptr radar_pc(new PointCloud<PointXYZI>);
    pcl::fromROSMsg(*pc_msg, *radar_pc);
    logdB_SNR(radar_pc);

    gtsam::Pose3 cp = toPose3(*odom);
    bool status = tracker.update(cp, radar_pc);

    if (!status)
        return;

    // visualization  =================================================================
    visualization_msgs::Marker marker;
    for (auto l : tracker.get_tracked())
    {
        geometry_msgs::Point pt;
        pt.x = l.p.x();
        pt.y = l.p.y();
        pt.z = l.p.z();
        marker.points.push_back(pt);
        marker.colors.push_back(id_to_color(l.id));
    }
    marker.header.frame_id = "radar";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1;
    marker.pose.orientation.w = 1;
    pub_landmark.publish(marker);

    sensor_msgs::PointCloud2 segment_msg;
    toROSMsg(*tracker.get_segment(), segment_msg);
    segment_msg.header.frame_id = "radar";
    pub_segment.publish(segment_msg);
}

std_msgs::ColorRGBA Clustering::id_to_color(uint id)
{
    std_msgs::ColorRGBA c;
    uint idx = id % colors.size();
    c.r = colors[idx][0];
    c.g = colors[idx][1];
    c.b = colors[idx][2];
    c.a = 1;
    return c;
}

Clustering::~Clustering()
{
}

int main(int argc, char *argv[])
{
    init(argc, argv, "radar_landmark_detect");
    NodeHandle nh;
    Clustering cls(&nh);
    spin();
    return 0;
}