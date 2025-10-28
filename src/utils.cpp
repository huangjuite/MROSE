#include "utils.h"
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <yaml-cpp/yaml.h>

namespace rplrio {

visualization_msgs::msg::Marker velocityMarker(gtsam::Vector3 velocity,
                                               gtsam::Vector3 position,
                                               std::string frame_id,
                                               std::string ns) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  geometry_msgs::msg::Point init_p, v_p;
  init_p.x = position(0);
  init_p.y = position(1);
  init_p.z = position(2);
  v_p.x = velocity(0) + position(0);
  v_p.y = velocity(1) + position(1);
  v_p.z = velocity(2) + position(2);
  marker.points.push_back(init_p);
  marker.points.push_back(v_p);
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1;
  marker.color.a = 1;

  return marker;
}

std::array<double, 36> toROSCov(Eigen::MatrixXd cov) {
  std::array<double, 36> rosCov;
  Eigen::MatrixXd cov_matrix = Eigen::Matrix<double, 6, 6>();
  cov_matrix.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);
  cov_matrix.block(3, 3, 3, 3) = cov.block(0, 0, 3, 3);
  cov_matrix.block(0, 3, 3, 3) = cov.block(3, 0, 3, 3);
  cov_matrix.block(3, 0, 3, 3) = cov.block(0, 3, 3, 3);
  double *data = cov_matrix.data();
  for (uint i = 0; i < 36; i++)
    rosCov[i] = data[i];

  return rosCov;
}

Eigen::MatrixXd fromROSCov(std::array<double, 36> ros_cov) {
  Eigen::MatrixXd cov_matrix = Eigen::Matrix<double, 6, 6>();
  Eigen::MatrixXd cov = Eigen::Matrix<double, 6, 6>();
  double *data = cov.data();
  for (uint i = 0; i < 36; i++)
    data[i] = ros_cov[i];

  cov_matrix.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);
  cov_matrix.block(3, 3, 3, 3) = cov.block(0, 0, 3, 3);
  cov_matrix.block(0, 3, 3, 3) = cov.block(3, 0, 3, 3);
  cov_matrix.block(3, 0, 3, 3) = cov.block(0, 3, 3, 3);

  return cov_matrix;
}

Eigen::MatrixXd fromROSCov_2d(std::array<double, 36> ros_cov) {
  Eigen::MatrixXd cov = Eigen::Matrix<double, 6, 6>();
  double *data = cov.data();
  for (uint i = 0; i < 36; i++)
    data[i] = ros_cov[i];

  Eigen::MatrixXd cov2d = cov.block(0, 0, 3, 3);
  cov2d.block(0, 2, 2, 1) = cov.block(0, 5, 2, 1);
  cov2d.block(2, 0, 1, 2) = cov.block(5, 0, 1, 2);
  cov2d(2, 2) = cov(5, 5);

  return cov2d;
}

nav_msgs::msg::Odometry toOdometry(Eigen::Matrix4d global_pose,
                                   gtsam::Matrix covariance) {
  nav_msgs::msg::Odometry odom;
  Eigen::Matrix3d orien = global_pose.block(0, 0, 3, 3);
  Eigen::Quaterniond qd(orien);

  odom.pose.pose.position.x = global_pose(0, 3);
  odom.pose.pose.position.y = global_pose(1, 3);
  odom.pose.pose.position.z = global_pose(2, 3);
  odom.pose.pose.orientation.x = qd.x();
  odom.pose.pose.orientation.y = qd.y();
  odom.pose.pose.orientation.z = qd.z();
  odom.pose.pose.orientation.w = qd.w();

  if (covariance.size() == 36) {
    odom.pose.covariance = toROSCov(covariance);
  }

  return odom;
}

geometry_msgs::msg::TwistWithCovarianceStamped
toROSTwist(gtsam::Vector3 v, gtsam::Matrix cov, std_msgs::msg::Header header) {
  geometry_msgs::msg::TwistWithCovarianceStamped twist;
  twist.header = header;
  twist.twist.twist.linear.x = v.x();
  twist.twist.twist.linear.y = v.y();
  twist.twist.twist.linear.z = v.z();

  for (uint i = 0; i < 3; i++)
    for (uint j = 0; j < 3; j++)
      twist.twist.covariance[i * 6 + j] = cov(i, j);

  return twist;
}

geometry_msgs::msg::Twist toROStwist(gtsam::imuBias::ConstantBias bias) {
  geometry_msgs::msg::Twist twist;
  gtsam::Vector3 acc = bias.accelerometer();
  gtsam::Vector3 gyr = bias.gyroscope();
  twist.linear.x = acc(0);
  twist.linear.y = acc(1);
  twist.linear.z = acc(2);
  twist.angular.x = gyr(0);
  twist.angular.y = gyr(1);
  twist.angular.z = gyr(2);
  return twist;
}

gtsam::Pose3 toPose3(const gtsam::Pose2 gtsam_pose) {
  return gtsam::Pose3(gtsam::Rot3::Yaw(gtsam_pose.theta()),
                      gtsam::Point3(gtsam_pose.x(), gtsam_pose.y(), 0));
}

gtsam::Pose3 toPose3(const nav_msgs::msg::Odometry odom) {
  return gtsam::Pose3(
      gtsam::Rot3(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                  odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
      gtsam::Point3(odom.pose.pose.position.x, odom.pose.pose.position.y,
                    odom.pose.pose.position.z));
}

gtsam::Pose3 toPose3(const geometry_msgs::msg::PoseStamped pst) {
  return gtsam::Pose3(
      gtsam::Rot3(pst.pose.orientation.w, pst.pose.orientation.x,
                  pst.pose.orientation.y, pst.pose.orientation.z),
      gtsam::Point3(pst.pose.position.x, pst.pose.position.y,
                    pst.pose.position.z));
}

geometry_msgs::msg::Point toROSPoint(gtsam::Point3 gtsam_point) {
  geometry_msgs::msg::Point point;
  point.x = gtsam_point.x();
  point.y = gtsam_point.y();
  point.z = gtsam_point.z();
  return point;
}

geometry_msgs::msg::Point toROSPoint(gtsam::Point2 gtsam_point) {
  geometry_msgs::msg::Point point;
  point.x = gtsam_point.x();
  point.y = gtsam_point.y();
  point.z = 0;
  return point;
}

geometry_msgs::msg::Pose toROSPose(gtsam::Pose3 gtsam_pose) {
  geometry_msgs::msg::Pose pose;
  auto trans = gtsam_pose.translation();
  auto rotat = gtsam_pose.rotation().toQuaternion();
  pose.position.x = trans.x();
  pose.position.y = trans.y();
  pose.position.z = trans.z();
  pose.orientation.w = rotat.w();
  pose.orientation.x = rotat.x();
  pose.orientation.y = rotat.y();
  pose.orientation.z = rotat.z();
  return pose;
}

geometry_msgs::msg::Pose toROSPose(gtsam::Pose2 gtsam_pose) {
  return toROSPose(toPose3(gtsam_pose));
}

geometry_msgs::msg::PoseStamped toROSPoseStamped(const gtsam::Pose3 gtsam_pose,
                                                 std_msgs::msg::Header header) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.pose = toROSPose(gtsam_pose);
  return pose_stamped;
}

geometry_msgs::msg::PoseStamped toROSPoseStamped(const gtsam::Pose2 gtsam_pose,
                                                 std_msgs::msg::Header header) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.pose = toROSPose(toPose3(gtsam_pose));
  return pose_stamped;
}

geometry_msgs::msg::TransformStamped
toROSTransform(const gtsam::Pose3 gtsam_pose, std::string frame_id,
               std::string child_id, std_msgs::msg::Header::_stamp_type stamp) {
  geometry_msgs::msg::TransformStamped tr;
  tr.header.stamp = stamp;
  tr.header.frame_id = frame_id;
  tr.child_frame_id = child_id;
  auto trans = gtsam_pose.translation();
  auto rotat = gtsam_pose.rotation().toQuaternion();
  tr.transform.translation.x = trans.x();
  tr.transform.translation.y = trans.y();
  tr.transform.translation.z = trans.z();
  tr.transform.rotation.w = rotat.w();
  tr.transform.rotation.x = rotat.x();
  tr.transform.rotation.y = rotat.y();
  tr.transform.rotation.z = rotat.z();
  return tr;
}

pcl::PointCloud<pcl::PointXYZSD>
from_xrio_type(pcl::PointCloud<pcl::PointXYZSNV> &radar_xrio) {
  pcl::PointCloud<pcl::PointXYZSD> radar_rpl;
  radar_rpl.resize(radar_xrio.size());
  for (size_t i = 0; i < radar_xrio.points.size(); i++) {
    radar_rpl.points[i].x = radar_xrio.points[i].x;
    radar_rpl.points[i].y = radar_xrio.points[i].y;
    radar_rpl.points[i].z = radar_xrio.points[i].z;
    radar_rpl.points[i].snr = radar_xrio.points[i].snr_db;
    radar_rpl.points[i].doppler = radar_xrio.points[i].v_doppler_mps;
  }

  return radar_rpl;
}

void logdB_SNR(pcl::PointCloud<pcl::PointXYZSD>::Ptr pc) {
  for (size_t i = 0; i < pc->size(); i++) {
    pc->points[i].snr = log(pc->points[i].snr);
  }
}

void logdB_SNR(pcl::PointCloud<pcl::PointXYZSDW>::Ptr pc) {
  for (size_t i = 0; i < pc->size(); i++) {
    pc->points[i].snr = log(pc->points[i].snr);
  }
}

void logdB_SNR(pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
  for (size_t i = 0; i < pc->size(); i++) {
    pc->points[i].intensity = log(pc->points[i].intensity);
  }
}

builtin_interfaces::msg::Time create_stamp_from_seconds(double seconds) {
  builtin_interfaces::msg::Time time_msg;
  time_msg.sec = static_cast<int32_t>(seconds);
  time_msg.nanosec = static_cast<uint32_t>(std::fmod(seconds, 1.0) * 1e9);
  return time_msg;
}

Params get_params(std::string cfg_path) {
  Params param;
  YAML::Node config = YAML::LoadFile(cfg_path);

  // imu
  std::string imu_;
  imu_ = config["odom"]["imu"].as<std::string>();
  param.imu_topic = config[imu_]["rostopic"].as<std::string>();
  param.acc_noise = config[imu_]["accelerometer_noise_density"].as<double>();
  param.acc_rw = config[imu_]["accelerometer_random_walk"].as<double>();
  param.gyro_noise = config[imu_]["gyroscope_noise_density"].as<double>();
  param.gyro_rw = config[imu_]["gyroscope_random_walk"].as<double>();
  param.gravity = config[imu_]["gravity"].as<double>();

  // Load 4x4 matrix T_i_b from YAML
  auto T_i_b_node = config[imu_]["T_i_b"];
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      param.T_i_b(i, j) = T_i_b_node[i][j].as<double>();
    }
  }
  param.R_i_b = param.T_i_b.block(0, 0, 3, 3);

  // radar
  param.radar_list = config["odom"]["radar"].as<std::vector<std::string>>();
  for (std::string r : param.radar_list) {
    param.radars[r].topic = config[r]["rostopic"].as<std::string>();
    param.radars[r].doppler_std = config[r]["doppler_sigma"].as<double>();

    auto T_r_i_node = config[r]["T_r_i"];
    for (size_t i = 0; i < 4; ++i) {
      for (size_t j = 0; j < 4; ++j) {
        param.radars[r].T_r_i(i, j) = T_r_i_node[i][j].as<double>();
      }
    }
    param.radars[r].R_r_i = param.radars[r].T_r_i.block(0, 0, 3, 3);
  }

  // odom
  param.lag = config["odom"]["optimization_window"].as<double>();
  param.bv_use_huber = config["odom"]["bv_use_huber"].as<bool>();
  param.bv_huber_k = config["odom"]["bv_huber_k"].as<double>();

  // initializer
  param.use_static_init = config["odom"]["use_static_init"].as<bool>();
  param.doppler_inlier_thresh =
      config["odom"]["doppler_inlier_thresh"].as<double>();
  param.init_window = config["odom"]["static_init_window"].as<double>();
  param.init_thres = config["odom"]["static_init_threshold"].as<double>();

  // mapping
  param.process_voxel_size =
      config["mapping"]["process_voxel_size"].as<double>();
  param.map_snr_thresh = config["mapping"]["map_snr_thresh"].as<double>();
  param.render_rate = config["mapping"]["render_rate"].as<double>();

  return param;
}

} // namespace rplrio
