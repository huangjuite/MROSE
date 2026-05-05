#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>

#include "core/RadarPointType.hpp"
#include "core/params.hpp"

namespace rplrio {

struct IMU {
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
  double stamp;

  IMU() {}
  IMU(Eigen::Vector3d a, Eigen::Vector3d g, double t) {
    acc = a;
    gyro = g;
    stamp = t;
  }

  static bool interpolate(IMU &imu, double time, IMU imu1, IMU imu2) {
    if (time > imu2.stamp || time < imu1.stamp || imu1.stamp > imu2.stamp)
      return false;

    if (abs(time - imu1.stamp) < 1e-6 || abs(time - imu2.stamp) < 1e-6)
      return false;

    double ratio = (time - imu1.stamp) / (imu2.stamp - imu1.stamp);
    imu.stamp = ratio * (imu2.stamp - imu1.stamp) + imu1.stamp;
    imu.acc = ratio * (imu2.acc - imu1.acc) + imu1.acc;
    imu.gyro = ratio * (imu2.gyro - imu1.gyro) + imu1.gyro;

    return true;
  }
};

struct Odom {
  double stamp;
  uint seq;
  gtsam::Pose3 pose;
  Eigen::Vector3d v;
  Eigen::Vector3d a;

  Odom(double t, uint s, gtsam::Pose3 p, Eigen::Vector3d lv,
       Eigen::Vector3d av) {
    stamp = t;
    seq = s;
    pose = p;
    v = lv;
    a = av;
  }

  bool operator<(double right) { return stamp < right; }
};

struct Radar {
  std::string frame_id;
  double stamp;
  RadarParams param;

  Eigen::Vector3d vel;
  Eigen::Matrix3d vel_cov;

  PointCloudPtr pc;

  Radar(PointCloudPtr p, double t, RadarParams params, std::string name) {
    stamp = t;
    pc = p;
    param = params;
    this->frame_id = name;
  }

  Radar(Eigen::Vector3d v, Eigen::Matrix3d cov, double t, RadarParams params,
        std::string name) {
    stamp = t;
    param = params;
    vel = v;
    vel_cov = cov;
    this->frame_id = name;
  }
};

struct Keyframe {
  double stamp;
  uint seq;
  std::string name;

  gtsam::NavState state;
  Eigen::MatrixXd pose_cov;
  Eigen::Matrix3d velocity_cov;
  Eigen::Vector3d v_err;

  IMU imu;
  gtsam::imuBias::ConstantBias imu_bias;
  Eigen::Matrix<double, 6, 6> imu_bias_cov;

  PointCloudPtr radar_points;
  Eigen::Vector3d radar_velocity;
  Eigen::Matrix3d radar_velocity_cov;
};

} // namespace rplrio
