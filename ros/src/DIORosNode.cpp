#include "DIORosNode.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "RosPointType.hpp"
#include "ros_params.hpp"
#include "utils.hpp"
#include "velocity/RadarVelocityEstimatorCeres.hpp"

DIORosNode::DIORosNode()
    : rclcpp::Node("radar_odometry",
                   rclcpp::NodeOptions()
                       .allow_undeclared_parameters(true)
                       .automatically_declare_parameters_from_overrides(true)) {
  declare_parameter("config", std::string("config.yaml"));
  auto config_path = get_parameter("config").as_string();
  RCLCPP_INFO(get_logger(), "config: %s", config_path.c_str());
  ros_params = get_ros_params(config_path);

  dio_manager = std::make_unique<rplrio::DIOManager>(
      ros_params.core, std::make_shared<RadarVelocityEstimatorCeres>());

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
      ros_params.imu_topic, 400,
      std::bind(&DIORosNode::cb_imu, this, std::placeholders::_1));

  for (const auto &r : ros_params.core.radar_list) {
    const auto &rrp = ros_params.ros_radars.at(r);
    if (rrp.type == "TwistWithCovarianceStamped") {
      sub_vel.push_back(this->create_subscription<
                        geometry_msgs::msg::TwistWithCovarianceStamped>(
          rrp.topic, 20,
          std::bind(&DIORosNode::cb_vel, this, std::placeholders::_1)));
    } else {
      sub_radars.push_back(
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              rrp.topic, 20,
              std::bind(&DIORosNode::cb_radar, this, std::placeholders::_1)));
    }
  }

  br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "radar odometry node initialized.");
}

void DIORosNode::publish_all(
    const std::shared_ptr<rplrio::Keyframe> &keyframe) {
  auto current_pose = keyframe->state.pose();
  auto T_i_b =
      gtsam::Pose3(gtsam::Rot3(ros_params.core.R_i_b),
                   gtsam::Point3(ros_params.core.T_i_b.block<3, 1>(0, 3)));
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
    if (keyframe->radar_points && !keyframe->radar_points->empty()) {
      const Eigen::Matrix4d &T = T_i_b.matrix();
      pcl::PointCloud<pcl::PointXYZ> inlier_xyz;
      inlier_xyz.reserve(keyframe->radar_points->size());
      for (const auto &p : *keyframe->radar_points) {
        Eigen::Vector4d pt(p.x, p.y, p.z, 1.0);
        Eigen::Vector4d pt_t = T * pt;
        inlier_xyz.push_back(pcl::PointXYZ(pt_t[0], pt_t[1], pt_t[2]));
      }
      sensor_msgs::msg::PointCloud2 inlier_msg;
      pcl::toROSMsg(inlier_xyz, inlier_msg);
      inlier_msg.header.stamp = ros_stamp;
      inlier_msg.header.frame_id = "base_link";
      pub_radar->publish(inlier_msg);
    }
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
    gtsam::Vector3 verr = keyframe->v_err;
    v_err.linear.x = verr(0);
    v_err.linear.y = verr(1);
    v_err.linear.z = verr(2);
    pub_err->publish(v_err);
  }

  if (pub_bias->get_subscription_count() > 0) {
    pub_bias->publish(rplrio::toROStwist(keyframe->imu_bias));
  }

  geometry_msgs::msg::TransformStamped transform_stamped =
      rplrio::toROSTransform(current_pose, "map", "base_link", ros_stamp);
  br->sendTransform(transform_stamped);
}

void DIORosNode::cb_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
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

void DIORosNode::cb_radar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg) {
  double radar_t = rclcpp::Time(pc_msg->header.stamp).seconds();

  pcl::PointCloud<pcl::PointXYZDoppler> pcl_cloud;
  pcl::fromROSMsg(*pc_msg, pcl_cloud);

  if (pcl_cloud.empty()) {
    RCLCPP_WARN(get_logger(), "%s msg has 0 points",
                pc_msg->header.frame_id.c_str());
    return;
  }

  auto cloud = std::make_shared<rplrio::PointCloud>();
  cloud->reserve(pcl_cloud.size());
  for (const auto &p : pcl_cloud) {
    cloud->push_back({p.x, p.y, p.z, p.doppler});
  }

  dio_manager->feed_radar(std::make_shared<rplrio::Radar>(
      cloud, radar_t, ros_params.core.radars[pc_msg->header.frame_id],
      pc_msg->header.frame_id));

  const auto &states = dio_manager->get_latest_states();
  for (const auto &kf : states) {
    publish_all(kf);
  }
}

void DIORosNode::cb_vel(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
        vel_msg) {
  double vel_t = rclcpp::Time(vel_msg->header.stamp).seconds();

  Eigen::Vector3d velocity(vel_msg->twist.twist.linear.x,
                           vel_msg->twist.twist.linear.y,
                           vel_msg->twist.twist.linear.z);

  Eigen::Matrix3d covariance;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      covariance(i, j) = vel_msg->twist.covariance[i * 6 + j];
    }
  }

  dio_manager->feed_radar(std::make_shared<rplrio::Radar>(
      velocity, covariance, vel_t,
      ros_params.core.radars[vel_msg->header.frame_id],
      vel_msg->header.frame_id));

  const auto &states = dio_manager->get_latest_states();
  for (const auto &kf : states) {
    publish_all(kf);
  }
}

DIORosNode::~DIORosNode() {}
