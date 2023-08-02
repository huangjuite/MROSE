
// std includes
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

// ROS includes
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

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

#include "radar_odom/Prediction.h"
#include "DIOManager.h"

#include "RadarVelocityEstimatorCeres.h"
#include "RadarVelocityEstimatorCeresHCS.h"
#include "RadarVelocityEstimatorCeresWeighted.h"
#include "RadarVelocityEstimatorEigenWeighted.h"

#include "rplrio_types.h"
#include "utils.h"
#include "params.h"

typedef pcl::PointXYZISW PointType;
typedef RadarVelocityEstimatorEigenWeighted VestType;

// typedef pcl::PointXYZIS PointType;
// typedef RadarVelocityEstimatorCeres VestType;

class DopplerIOBag
{
private:
    rplrio::Params params;
    std::unique_ptr<rplrio::DIOManager<PointType>> dio_manager;

    ros::NodeHandle *nh;

    ros::Publisher pub_odom;
    ros::Publisher pub_v;
    ros::Publisher pub_twist;
    ros::Publisher pub_bias;
    ros::Publisher pub_radar;
    ros::Publisher pub_error;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    void publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe);

public:
    DopplerIOBag(ros::NodeHandle *nh);
    ~DopplerIOBag();

    void cb_radar(const sensor_msgs::PointCloud2 &pc_msg);
    void cb_imu(const sensor_msgs::Imu::ConstPtr &imu);
};

DopplerIOBag::DopplerIOBag(ros::NodeHandle *nh)
{
    params = rplrio::get_params(*nh);
    std::cout << params.acc_noise << std::endl;
    dio_manager = std::make_unique<rplrio::DIOManager<PointType>>(
        params, std::make_shared<VestType>());

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/velocity", 1);
    pub_bias = nh->advertise<geometry_msgs::Twist>("radar/bias", 1);
    pub_error = nh->advertise<geometry_msgs::Twist>("radar/error", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
}

void DopplerIOBag::publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe)
{
    auto current_time = ros::Time::now();
    nav_msgs::Odometry odom = toOdometry(keyframe->state.pose().matrix(), keyframe->pose_cov);
    odom.header.stamp = current_time;
    odom.header.seq = keyframe->seq;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*(keyframe->radar_points), inlier_msg);
    inlier_msg.header.stamp = current_time;
    inlier_msg.header.frame_id = "base";
    inlier_msg.header.seq = keyframe->seq;
    geometry_msgs::TwistWithCovarianceStamped radar_v =
        toROSTwist(keyframe->radar_velocity, keyframe->radar_velocity_cov, inlier_msg.header);
    radar_v.header.seq = keyframe->seq;
    radar_v.header.frame_id = "base";

    geometry_msgs::Twist v_err;
    gtsam::Vector3 verr = keyframe->state.bodyVelocity() - keyframe->radar_velocity;
    v_err.linear.x = verr(0);
    v_err.linear.y = verr(1);
    v_err.linear.z = verr(2);
    pub_error.publish(v_err);

    pub_radar.publish(inlier_msg);
    pub_twist.publish(radar_v);
    pub_bias.publish(toROStwist(keyframe->imu_bias));
    pub_v.publish(velocityMarker(
        keyframe->state.velocity(),
        keyframe->state.pose().translation(),
        "map",
        "velocity"));

    // broadcast odometry transformation
    // tf::Transform transform;
    // auto trans = prev_state.pose().translation();
    // auto orien = prev_state.pose().rotation().toQuaternion();
    // transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    // transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    // br.sendTransform(tf::StampedTransform(transform, radar_msg->header.stamp, "map", "base"));
}

void DopplerIOBag::cb_imu(const sensor_msgs::Imu::ConstPtr &imu_msg)
{

    double imu_t = imu_msg->header.stamp.toSec();

    gtsam::Vector3 acc = params.R_i_b * (gtsam::Vector3() << imu_msg->linear_acceleration.x,
                                         imu_msg->linear_acceleration.y,
                                         imu_msg->linear_acceleration.z)
                                            .finished();
    gtsam::Vector3 omega = params.R_i_b * (gtsam::Vector3() << imu_msg->angular_velocity.x,
                                           imu_msg->angular_velocity.y,
                                           imu_msg->angular_velocity.z)
                                              .finished();

    dio_manager->feed_imu(std::make_shared<rplrio::IMU>(acc, omega, imu_t));
}

void DopplerIOBag::cb_radar(const sensor_msgs::PointCloud2 &pc_msg)
{
    pcl::PointCloud<PointType>::Ptr radar_pc;
    radar_pc.reset(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(pc_msg, *radar_pc);
    double radar_t = pc_msg.header.stamp.toSec();

    dio_manager->feed_radar(std::make_shared<rplrio::Radar<PointType>>(radar_pc, radar_t));

    auto states = dio_manager->get_latest_states();
    for (auto kf : states)
    {
        publish_all(kf);
    }
}

DopplerIOBag::~DopplerIOBag()
{
}

int main(int argc, char *argv[])
{
    std::cout << std::fixed << std::setprecision(6);
    ros::init(argc, argv, "doppler_inertial_odom");
    ros::NodeHandle nh;

    DopplerIOBag runner(&nh);
    auto predict_noise = nh.serviceClient<radar_odom::Prediction>("radar_uncertainty");

    std::string bag_folder;
    ros::param::get("~bag_folder", bag_folder);
    std::cout << bag_folder << std::endl;

    rplrio::Params params = rplrio::get_params(nh);
    std::vector<Eigen::Vector3d> body_velocity, radar_velocity;
    std::vector<double> stamps;

    std::vector<std::string> bags;
    std::vector<std::string> topics;
    for (const auto &entry : std::filesystem::directory_iterator(bag_folder))
        bags.emplace_back(entry.path().string());
    std::sort(bags.begin(), bags.end());
    topics.push_back(params.radars[params.radar_list[0]].topic);
    topics.push_back(params.imu_topic);

    std::cout << "input any to start:\n";
    std::string tmp;
    std::cin >> tmp;

    for (auto b : bags)
    {
        rosbag::Bag bag;
        bag.open(b, rosbag::bagmode::Read);
        std::cout << b << std::endl;

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        foreach (rosbag::MessageInstance const m, view)
        {
            sensor_msgs::PointCloud2::ConstPtr radar_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (radar_msg != NULL)
            {

                // runner.cb_radar(*radar_msg);

                radar_odom::Prediction req;
                req.request.input = *radar_msg;
                if (predict_noise.call(req))
                {
                    runner.cb_radar(req.response.output);
                }
            }

            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg != NULL)
            {
                runner.cb_imu(imu_msg);
            }
        }

        bag.close();
    }

    return 0;
}