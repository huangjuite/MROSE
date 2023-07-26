#include "DopplerInertialOdom.h"

DopplerIO::DopplerIO(ros::NodeHandle *nh)
{
    params = rplrio::get_params(*nh);
    dio_manager = std::make_unique<rplrio::DIOManager<PointType>>(params);

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/velocity", 1);
    pub_bias = nh->advertise<geometry_msgs::Twist>("radar/bias", 1);
    pub_error = nh->advertise<geometry_msgs::Twist>("radar/error", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    sub_radar = nh->subscribe(params.radar_topic, 1, &DopplerIO::cb_radar, this);
    sub_imu = nh->subscribe(params.imu_topic, 1000, &DopplerIO::cb_imu, this);
}

void DopplerIO::publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe)
{

    nav_msgs::Odometry odom = toOdometry(keyframe->state.pose().matrix(), keyframe->pose_cov);
    odom.header.stamp = ros::Time().fromSec(keyframe->stamp);
    odom.header.seq = keyframe->seq;
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    sensor_msgs::PointCloud2 inlier_msg;
    pcl::toROSMsg(*(keyframe->radar_points), inlier_msg);
    inlier_msg.header.stamp = ros::Time().fromSec(keyframe->stamp);
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

void DopplerIO::cb_imu(const sensor_msgs::Imu::ConstPtr &imu_msg)
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

void DopplerIO::cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    pcl::PointCloud<PointType>::Ptr radar_pc;
    radar_pc.reset(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*pc_msg, *radar_pc);
    double radar_t = pc_msg->header.stamp.toSec();

    dio_manager->feed_radar(std::make_shared<rplrio::Radar<PointType>>(radar_pc, radar_t));

    auto states = dio_manager->get_latest_states();
    for (auto kf : states){
        publish_all(kf);
    }
}

DopplerIO::~DopplerIO()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "doppler_inertial_odom");
    ros::NodeHandle nh;
    DopplerIO dio(&nh);
    ros::spin();

    return 0;
}