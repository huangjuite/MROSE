#include "RosNode/DIORosNode.h"
#include "RadarPointType.h"
#include "utils.h"

template <class PointType>
DIORosNode<PointType>::DIORosNode()
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
  pub_twist =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "rio/velocity", 1);
  pub_bias =
      this->create_publisher<geometry_msgs::msg::Twist>("rio/imu_bias", 1);
  pub_err =
      this->create_publisher<geometry_msgs::msg::Twist>("rio/residual", 1);
  pub_radar =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rio/inlier", 1);
  pub_pose =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("rio/pose", 1);
  pub_path = this->create_publisher<nav_msgs::msg::Path>("rio/path", 1);

  path_msg.header.frame_id = "map";

  sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      params.imu_topic, 400,
      std::bind(&DIORosNode::cb_imu, this, std::placeholders::_1));

  for (auto r : params.radar_list) {
    sub_radars.push_back(
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            params.radars[r].topic, 20,
            std::bind(&DIORosNode::cb_radar, this, std::placeholders::_1)));
  }

  br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "radar odometry node initialized.");
}

template <class PointType>
void DIORosNode<PointType>::publish_all(
    const std::shared_ptr<rplrio::Keyframe<PointType>> keyframe) {

  auto current_pose = keyframe->state.pose();
  auto T_i_b = gtsam::Pose3(gtsam::Rot3(params.R_i_b),
                            gtsam::Point3(params.T_i_b.block<3, 1>(0, 3)));
  current_pose = current_pose.compose(T_i_b);
  auto ros_stamp = rplrio::create_stamp_from_seconds(keyframe->stamp);
  nav_msgs::msg::Odometry odom =
      rplrio::toOdometry(current_pose.matrix(), keyframe->pose_cov);
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = ros_stamp;
  pose_msg.header.frame_id = "map";
  pose_msg.pose = odom.pose.pose;
  path_msg.poses.push_back(pose_msg);

  if (pub_pose->get_subscription_count() > 0) {
    pub_pose->publish(pose_msg);
  }

  if (pub_path->get_subscription_count() > 0) {
    pub_path->publish(path_msg);
  }

  if (pub_radar->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 inlier_msg;
    pcl::transformPointCloud(*keyframe->radar_points, *keyframe->radar_points,
                             T_i_b.matrix());
    pcl::toROSMsg(*(keyframe->radar_points), inlier_msg);
    inlier_msg.header.stamp = ros_stamp;
    inlier_msg.header.frame_id = "base_link";
    pub_radar->publish(inlier_msg);
  }

  if (pub_twist->get_subscription_count() > 0) {
    geometry_msgs::msg::TwistWithCovarianceStamped radar_v = rplrio::toROSTwist(
        keyframe->radar_velocity, keyframe->radar_velocity_cov);
    radar_v.header.stamp = ros_stamp;
    radar_v.header.frame_id = imu_frame_id_;
    pub_twist->publish(radar_v);
  }

  if (pub_err->get_subscription_count() > 0) {
    geometry_msgs::msg::Twist v_err;
    gtsam::Vector3 verr =
        keyframe->state.bodyVelocity() - keyframe->radar_velocity;
    v_err.linear.x = verr(0);
    v_err.linear.y = verr(1);
    v_err.linear.z = verr(2);
    pub_err->publish(v_err);
  }

  if (pub_bias->get_subscription_count() > 0) {
    pub_bias->publish(rplrio::toROStwist(keyframe->imu_bias));
  }

  // broadcast transformation
  geometry_msgs::msg::TransformStamped transform_stamped =
      rplrio::toROSTransform(current_pose, "map", "base_link", ros_stamp);
  br->sendTransform(transform_stamped);
}

template <class PointType>
void DIORosNode<PointType>::cb_imu(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
  double imu_t = rclcpp::Time(imu_msg->header.stamp).seconds();

  if (imu_frame_id_.empty()) {
    imu_frame_id_ = imu_msg->header.frame_id;
  }

  gtsam::Vector3 acc =
      (gtsam::Vector3() << imu_msg->linear_acceleration.x,
       imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z)
          .finished();
  gtsam::Vector3 omega =
      (gtsam::Vector3() << imu_msg->angular_velocity.x,
       imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)
          .finished();

  dio_manager->feed_imu(std::make_shared<rplrio::IMU>(acc, omega, imu_t));
}

template <class PointType>
void DIORosNode<PointType>::cb_radar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {

  double radar_t = rclcpp::Time(pc_msg->header.stamp).seconds();

  PclPtr radar_pc;
  radar_pc.reset(new Pcl);
  pcl::fromROSMsg(*pc_msg, *radar_pc);

  if (radar_pc->empty()) {
    return;
  }

  dio_manager->feed_radar(std::make_shared<rplrio::Radar<PointType>>(
      radar_pc, radar_t, params.radars[pc_msg->header.frame_id]));

  const auto& states = dio_manager->get_latest_states();
  for (auto kf : states) {
    publish_all(kf);
  }
}

template <class PointType> DIORosNode<PointType>::~DIORosNode() {}

template class DIORosNode<pcl::PointXYZD>;
template class DIORosNode<pcl::PointXYZSD>;
template class DIORosNode<pcl::PointHugin>;
