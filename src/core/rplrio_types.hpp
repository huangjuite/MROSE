#pragma once

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include "core/params.hpp"
#include "core/RadarPointType.hpp"

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

  gtsam::Vector3 vel;
  gtsam::Matrix33 vel_cov;

  PointCloudPtr pc;

  Radar(PointCloudPtr p, double t, RadarParams params,
        std::string name) {
    stamp = t;
    pc = p;
    param = params;
    this->frame_id = name;
  }

  Radar(gtsam::Vector3 v, gtsam::Matrix33 cov, double t, RadarParams params,
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
  gtsam::Matrix pose_cov;
  gtsam::Matrix33 velocity_cov;
  gtsam::Vector3 v_err;

  IMU imu;
  gtsam::imuBias::ConstantBias imu_bias;
  gtsam::Matrix66 imu_bias_cov;

  PointCloudPtr radar_points;
  gtsam::Vector3 radar_velocity;
  gtsam::Matrix33 radar_velocity_cov;
};

} // namespace rplrio
