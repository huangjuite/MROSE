#include "RIO.h"

RadarIO::RadarIO(ros::NodeHandle *nh)
{
    params = rplrio::get_params(*nh);
    rio_manager = std::make_unique<rplrio::RIOManager<PointType>>(
        params, std::make_shared<VestType>());

    // ros init
    pub_v = nh->advertise<visualization_msgs::Marker>("velocity", 1);
    pub_twist = nh->advertise<geometry_msgs::TwistWithCovarianceStamped>("radar/velocity", 1);
    pub_bias = nh->advertise<geometry_msgs::Twist>("radar/bias", 1);
    pub_error = nh->advertise<geometry_msgs::Twist>("radar/error", 1);
    pub_odom = nh->advertise<nav_msgs::Odometry>("dopplerIO", 1);
    pub_radar = nh->advertise<sensor_msgs::PointCloud2>("radar_inlier", 1);
    pub_keyframe = nh->advertise<radar_odom::Keyframe>("radar/keyframe", 1);

    sub_imu = nh->subscribe(params.imu_topic, 1000, &RadarIO::cb_imu, this);
    sub_mappose = nh->subscribe("radar/map_pose", 5, &RadarIO::cb_mappose, this);
    for (auto r : params.radar_list)
    {
        sub_radars.emplace_back(
            nh->subscribe(params.radars[r].topic, 20, &RadarIO::cb_radar, this));
    }
}

void RadarIO::publish_all(const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe)
{

    nav_msgs::Odometry odom = toOdometry(keyframe->state.pose().matrix(), keyframe->pose_cov);
    odom.header.stamp = ros::Time().fromSec(keyframe->stamp);
    odom.header.frame_id = "map";
    pub_odom.publish(odom);

    radar_odom::Keyframe key_msg;
    pcl::toROSMsg(*(keyframe->radar_points), key_msg.radar_points);
    key_msg.radar_points.header.stamp = ros::Time().fromSec(keyframe->stamp);
    key_msg.radar_points.header.frame_id = "base";
    key_msg.pose = toROSPoseStamped(keyframe->state.pose(), odom.header);
    key_msg.seq = keyframe->seq;
    pub_keyframe.publish(key_msg);

    if (pub_radar.getNumSubscribers() > 0)
    {
        pub_radar.publish(key_msg.radar_points);
    }

    if (pub_twist.getNumSubscribers() > 0)
    {
        geometry_msgs::TwistWithCovarianceStamped radar_v =
            toROSTwist(keyframe->radar_velocity, keyframe->radar_velocity_cov, odom.header);
        radar_v.header.seq = keyframe->seq;
        radar_v.header.frame_id = "base";
        pub_twist.publish(radar_v);
    }

    if (pub_error.getNumSubscribers() > 0)
    {
        geometry_msgs::Twist v_err;
        gtsam::Vector3 verr = keyframe->state.bodyVelocity() - keyframe->radar_velocity;
        v_err.linear.x = verr(0);
        v_err.linear.y = verr(1);
        v_err.linear.z = verr(2);
        pub_error.publish(v_err);
    }

    if (pub_bias.getNumSubscribers() > 0)
        pub_bias.publish(toROStwist(keyframe->imu_bias));

    if (pub_v.getNumSubscribers() > 0)
        pub_v.publish(velocityMarker(
            keyframe->state.velocity(),
            keyframe->state.pose().translation(),
            "map",
            "velocity"));

    // broadcast odometry transformation
    tf::Transform transform;
    auto trans = keyframe->state.pose().translation();
    auto orien = keyframe->state.pose().rotation().toQuaternion();
    transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
    transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(), orien.w()));
    br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "base"));
}

void RadarIO::cb_imu(const sensor_msgs::Imu::ConstPtr &imu_msg)
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

    rio_manager->feed_imu(std::make_shared<rplrio::IMU>(acc, omega, imu_t));
}

void RadarIO::cb_radar(const sensor_msgs::PointCloud2::ConstPtr &pc_msg)
{
    pcl::PointCloud<PointType>::Ptr radar_pc;
    radar_pc.reset(new pcl::PointCloud<PointType>);

    pcl::fromROSMsg(*pc_msg, *radar_pc);
    double radar_t = pc_msg->header.stamp.toSec();

    rio_manager->feed_radar(std::make_shared<rplrio::Radar<PointType>>(
        radar_pc, radar_t, params.radars[pc_msg->header.frame_id]));

    auto states = rio_manager->get_latest_states();
    for (auto kf : states)
    {
        publish_all(kf);
    }
}

void RadarIO::cb_mappose(const radar_odom::KeyframeConstPtr &mappose)
{
    gtsam::Pose3 mp = toPose3(mappose->pose);
    rio_manager->feed_mappose(mp, mappose->seq);
}

RadarIO::~RadarIO()
{
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_inertial_odom");
    ros::NodeHandle nh;
    RadarIO dio(&nh);
    ros::spin();

    return 0;
}