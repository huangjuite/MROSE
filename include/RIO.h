
// std includes
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <deque>
#include <memory>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <radar_odom/Keyframe.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

#include "RIOManager.h"

#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorEigenWeighted.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

typedef pcl::PointXYZSD PointType;
typedef RadarVelocityEstimatorCeres VestType;

class RadarIO
{
private:
    rplrio::Params params;
    std::unique_ptr<rplrio::RIOManager<PointType>> rio_manager;

    ros::NodeHandle *nh;

    ros::Subscriber sub_imu;
    std::vector<ros::Subscriber> sub_radars;
    ros::Subscriber sub_mappose;

    ros::Publisher pub_odom;
    ros::Publisher pub_v;
    ros::Publisher pub_twist;
    ros::Publisher pub_bias;
    ros::Publisher pub_radar;
    ros::Publisher pub_error;
    ros::Publisher pub_subkey;
    ros::Publisher pub_submap;
    ros::Publisher pub_subkeypos;
    ros::Publisher pub_aligned;
    ros::Publisher pub_keyframe;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    void cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg);
    void cb_imu(const sensor_msgs::Imu::ConstPtr &imu);
    void cb_mappose(const radar_odom::KeyframeConstPtr &mappose);
    void publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe);

public:
    RadarIO(ros::NodeHandle *nh);
    ~RadarIO();
};
