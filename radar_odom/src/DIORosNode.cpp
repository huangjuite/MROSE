#include "DIORosNode.h"
#include "utils.h"
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>

DIORosNode::DIORosNode()
    : rclcpp::Node("radar_odometry",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {
  auto cfg =
      get_parameter_or("config", rclcpp::Parameter("config", "config.yaml"));
  RCLCPP_INFO(get_logger(), "config: %s", cfg.as_string().c_str());
  params = rplrio::get_params(cfg.as_string());
  dio_manager = std::make_unique<rplrio::DIOManager<PointType>>(
      params, std::make_shared<VestType>());

  // ros init
  pub_v =
      this->create_publisher<visualization_msgs::msg::Marker>("velocity", 1);
  pub_twist =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "radar/velocity", 1);
  pub_bias = this->create_publisher<geometry_msgs::msg::Twist>("radar/bias", 1);
  pub_error =
      this->create_publisher<geometry_msgs::msg::Twist>("radar/error", 1);
  pub_radar =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_inlier", 1);
  pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("dopplerIO", 1);

  sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      params.imu_topic, 10,
      std::bind(&DIORosNode::cb_imu, this, std::placeholders::_1));

  for (auto r : params.radar_list) {
    sub_radars.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params.radars[r].topic, 1,
            std::bind(&DIORosNode::cb_radar, this, std::placeholders::_1)));
  }
}

void DIORosNode::publish_all(
    const std::shared_ptr<const rplrio::Keyframe<PointType>> keyframe) {

  nav_msgs::msg::Odometry odom =
      rplrio::toOdometry(keyframe->state.pose().matrix(), keyframe->pose_cov);
  odom.header.stamp = rplrio::create_stamp_from_seconds(keyframe->stamp);
  odom.header.frame_id = "map";
  pub_odom->publish(odom);

  sensor_msgs::msg::PointCloud2 inlier_msg;
  pcl::toROSMsg(*(keyframe->radar_points), inlier_msg);
  inlier_msg.header.stamp = rplrio::create_stamp_from_seconds(keyframe->stamp);
  inlier_msg.header.frame_id = "base";
  geometry_msgs::msg::TwistWithCovarianceStamped radar_v =
      rplrio::toROSTwist(keyframe->radar_velocity, keyframe->radar_velocity_cov,
                         inlier_msg.header);
  radar_v.header.frame_id = "base";

  geometry_msgs::msg::Twist v_err;
  gtsam::Vector3 verr =
      keyframe->state.bodyVelocity() - keyframe->radar_velocity;
  v_err.linear.x = verr(0);
  v_err.linear.y = verr(1);
  v_err.linear.z = verr(2);
  pub_error->publish(v_err);

  pub_radar->publish(inlier_msg);
  pub_twist->publish(radar_v);
  pub_bias->publish(rplrio::toROStwist(keyframe->imu_bias));
  pub_v->publish(rplrio::velocityMarker(keyframe->state.velocity(),
                                        keyframe->state.pose().translation(),
                                        "map", "velocity"));

  // broadcast odometry transformation
  // tf::Transform transform;
  // auto trans = prev_state.pose().translation();
  // auto orien = prev_state.pose().rotation().toQuaternion();
  // transform.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));
  // transform.setRotation(tf::Quaternion(orien.x(), orien.y(), orien.z(),
  // orien.w())); br.sendTransform(tf::StampedTransform(transform,
  // radar_msg->header.stamp, "map", "base"));
}

void DIORosNode::cb_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {

  double imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

  gtsam::Vector3 acc =
      params.R_i_b * (gtsam::Vector3() << imu_msg->linear_acceleration.x,
                      imu_msg->linear_acceleration.y,
                      imu_msg->linear_acceleration.z)
                         .finished();
  gtsam::Vector3 omega =
      params.R_i_b * (gtsam::Vector3() << imu_msg->angular_velocity.x,
                      imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)
                         .finished();

  dio_manager->feed_imu(std::make_shared<rplrio::IMU>(acc, omega, imu_t));
}

void DIORosNode::cb_radar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  pcl::PointCloud<PointType>::Ptr radar_pc;
  radar_pc.reset(new pcl::PointCloud<PointType>);

  pcl::fromROSMsg(*pc_msg, *radar_pc);
  double radar_t = rclcpp::Time(pc_msg->header.stamp).seconds();

  dio_manager->feed_radar(std::make_shared<rplrio::Radar<PointType>>(
      radar_pc, radar_t, params.radars[pc_msg->header.frame_id]));

  auto states = dio_manager->get_latest_states();
  for (auto kf : states) {
    publish_all(kf);
  }
}

DIORosNode::~DIORosNode() {}
