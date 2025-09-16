// std includes
#include <string>
#include <vector>
#include <iostream>
#include <deque>
#include <memory>
#include <thread>

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pclradar/RadarPointType.h>

#include <gtsam/geometry/Pose3.h>

#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include <MapConstraint.h>
#include <radar_odom/Keyframe.h>
#include <radar_odom/GetMap.h>

#include "rplrio_types.h"
#include "utils.h"

template <class PointType>
class RIO_map
{
private:
    typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
    typedef typename pcl::PointCloud<PointType> Pcl;
    typedef std::shared_ptr<rplrio::Keyframe<PointType>> KeyFramePtr;

    MapConstraint<PointType> radar_map;

    nav_msgs::Path path;
    PclPtr map_;

    ros::Publisher pub_subkey, pub_submap, pub_subkeypos,
        pub_aligned, pub_mappose, pub_path, pub_map;

    ros::Subscriber sub_keyframe;

    ros::ServiceServer map_srv;
    bool getMap(radar_odom::GetMap::Request &req, radar_odom::GetMap::Response &resp);

    void cb_node(const radar_odom::KeyframeConstPtr &key_msg);

public:
    RIO_map(ros::NodeHandle *nh);
    ~RIO_map();

    void render_map();
};

template <class PointType>
RIO_map<PointType>::RIO_map(ros::NodeHandle *nh)
{
    map_.reset(new Pcl);

    pub_subkey = nh->advertise<sensor_msgs::PointCloud2>("radar/subkeyframe", 1);
    pub_submap = nh->advertise<sensor_msgs::PointCloud2>("radar/submap", 1);
    pub_subkeypos = nh->advertise<sensor_msgs::PointCloud2>("radar/subkeypos", 1);
    pub_aligned = nh->advertise<sensor_msgs::PointCloud2>("radar/aligned", 1);
    pub_map = nh->advertise<sensor_msgs::PointCloud2>("radar/map", 1);
    pub_mappose = nh->advertise<radar_odom::Keyframe>("radar/map_pose", 1);
    pub_path = nh->advertise<nav_msgs::Path>("radar_path", 1);

    path.header.frame_id = "map";

    map_srv = nh->advertiseService("radar/get_map", &RIO_map::getMap, this);

    sub_keyframe = nh->subscribe("radar/keyframe", 40, &RIO_map::cb_node, this);
}

template <class PointType>
RIO_map<PointType>::~RIO_map()
{
}

template <class PointType>
void RIO_map<PointType>::cb_node(const radar_odom::KeyframeConstPtr &key_msg)
{

    auto keyframe = std::make_shared<rplrio::Keyframe<pcl::PointXYZSD>>();
    keyframe->seq = key_msg->seq;
    keyframe->state = gtsam::NavState(toPose3(key_msg->pose), gtsam::Vector3());
    keyframe->radar_points.reset(new pcl::PointCloud<PointType>);
    fromROSMsg(key_msg->radar_points, *keyframe->radar_points);
    logdB_SNR(keyframe->radar_points);

    path.poses.push_back(toROSPoseStamped(keyframe->state.pose(), key_msg->pose.header));

    // ------------------------------------------------------------------------------------
    gtsam::Pose3 scm_pose;
    PclPtr subkeyframe, submap, aligned;
    pcl::PointCloud<pcl::PointXYZ>::Ptr subkeypos;

    radar_map.update_sub_key_frame(keyframe);

    if (keyframe->seq % 40 == 0)
    {
        subkeyframe.reset(new Pcl);
        submap.reset(new Pcl);
        aligned.reset(new Pcl);
        subkeypos.reset(new pcl::PointCloud<pcl::PointXYZ>);
        bool sc_regist = radar_map.regist(
            submap, subkeyframe, aligned, subkeypos, scm_pose);

        if (sc_regist)
        {
            radar_odom::Keyframe mappose_msg;
            mappose_msg.pose = toROSPoseStamped(scm_pose, key_msg->pose.header);
            mappose_msg.seq = keyframe->seq;
            pub_mappose.publish(mappose_msg);
        }
    }

    // ------------------------------------------------------------------------------------
    // visualize
    path.header.stamp = key_msg->pose.header.stamp;
    pub_path.publish(path);

    if (subkeyframe != nullptr && pub_subkey.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 subkey_msg;
        pcl::toROSMsg(*subkeyframe, subkey_msg);
        subkey_msg.header.stamp = ros::Time().fromSec(keyframe->stamp);
        subkey_msg.header.frame_id = "map";
        pub_subkey.publish(subkey_msg);
    }
    if (submap != nullptr && pub_submap.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 submap_msg;
        pcl::toROSMsg(*submap, submap_msg);
        submap_msg.header.stamp = ros::Time().fromSec(keyframe->stamp);
        submap_msg.header.frame_id = "map";
        pub_submap.publish(submap_msg);
    }
    if (aligned != nullptr && pub_aligned.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_msg.header.stamp = ros::Time().fromSec(keyframe->stamp);
        aligned_msg.header.frame_id = "map";
        pub_aligned.publish(aligned_msg);
    }
    if (subkeypos != nullptr && pub_subkeypos.getNumSubscribers() > 0)
    {
        sensor_msgs::PointCloud2 subpos_msg;
        pcl::toROSMsg(*subkeypos, subpos_msg);
        subpos_msg.header.stamp = ros::Time().fromSec(keyframe->stamp);
        subpos_msg.header.frame_id = "map";
        pub_subkeypos.publish(subpos_msg);
    }

    pcl::transformPointCloud(*keyframe->radar_points, *keyframe->radar_points,
                             keyframe->state.pose().matrix());
    *map_ += *keyframe->radar_points;
}

template <class PointType>
void RIO_map<PointType>::render_map()
{
    ros::Rate rate(1);
    while (ros::ok())
    {
        rate.sleep();
        if (pub_map.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud2 map_msg;
            pcl::toROSMsg(*map_, map_msg);
            map_msg.header.frame_id = "map";
            pub_map.publish(map_msg);
        }
    }
}

template <class PointType>
bool RIO_map<PointType>::getMap(radar_odom::GetMap::Request &req, radar_odom::GetMap::Response &resp)
{
    pcl::toROSMsg(*map_, resp.map);
    resp.map.header.frame_id = "map";

    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_map");
    ros::NodeHandle nh;
    RIO_map<pcl::PointXYZSD> rpg(&nh);

    std::thread map_thread(&RIO_map<pcl::PointXYZSD>::render_map, &rpg);

    ros::spin();
    return 0;
}