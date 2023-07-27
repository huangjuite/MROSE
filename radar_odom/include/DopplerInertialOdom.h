
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

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

#include "DIOManager.h"

#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorCeresHCS.h"
#include "RadarVelocityEstimatorCeresWeighted.h"
#include "RadarVelocityEstimatorEigenWeighted.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

typedef pcl::PointXYZIS PointType;
typedef RadarVelocityEstimatorCeres VestType;

class DopplerIO
{
private:
    rplrio::Params params;
    std::unique_ptr<rplrio::DIOManager<PointType>> dio_manager;

    ros::NodeHandle *nh;
    
    ros::Subscriber sub_radar;
    ros::Subscriber sub_imu;

    ros::Publisher pub_odom;
    ros::Publisher pub_v;
    ros::Publisher pub_twist;
    ros::Publisher pub_bias;
    ros::Publisher pub_radar;
    ros::Publisher pub_error;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    void cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg);
    void cb_imu(const sensor_msgs::Imu::ConstPtr &imu);
    void publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe);

public:
    DopplerIO(ros::NodeHandle *nh);
    ~DopplerIO();

};
