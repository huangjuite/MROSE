#include "StaticInitializer.h"
#include <spdlog/spdlog.h>

StaticInitializer::StaticInitializer(float init_window, float acc_var_thresh,
                                     float gravity)
    : init_window_(init_window), acc_var_thresh_(acc_var_thresh),
      gravity_(abs(gravity)) {
  logger_ = spdlog::get("rplrio");
}

Eigen::Quaterniond StaticInitializer::get_rotation() { return q_MtoI_; }
Eigen::Vector3d StaticInitializer::get_bias_acc() { return bias_acc_; }
Eigen::Vector3d StaticInitializer::get_bias_gyro() { return bias_gyro_; }

std::deque<std::shared_ptr<rplrio::IMU>> StaticInitializer::get_imu_queue() {
  size_t c_idx = imu_queue.size() / 2;
  std::deque<std::shared_ptr<rplrio::IMU>> unintialized(
      imu_queue.begin() + c_idx, imu_queue.end());
  return unintialized;
}

double StaticInitializer::get_first_stamp() {
  size_t c_idx = imu_queue.size() / 2;
  return imu_queue[c_idx - 1]->stamp;
}

Eigen::Matrix3d StaticInitializer::gram_schmidt(Eigen::Vector3d gravity) {
  Eigen::Matrix3d R_MinI;
  Eigen::Vector3d z_axis = gravity / gravity.norm();
  Eigen::Vector3d x_axis(1, 0, 0);
  x_axis = x_axis - x_axis.dot(z_axis) * z_axis;
  x_axis /= x_axis.norm();
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  y_axis /= y_axis.norm();

  R_MinI.block(0, 0, 3, 1) = x_axis;
  R_MinI.block(0, 1, 3, 1) = y_axis;
  R_MinI.block(0, 2, 3, 1) = z_axis;
  return R_MinI;
}

bool StaticInitializer::init_update(std::shared_ptr<rplrio::IMU> imu) {
  imu_queue.emplace_back(imu);
  double latest_time = imu->stamp;
  double oldest_time = imu_queue.front()->stamp;

  // not enough mearsurements
  if (imu_queue.size() < 2) {
    return false;
  }

  // not enough time
  if (latest_time - oldest_time < init_window_) {
    return false;
  }

  // prune
  while (true) {
    oldest_time = imu_queue.front()->stamp;
    if (latest_time - oldest_time < init_window_) {
      break;
    }
    imu_queue.pop_front();
  }

  // detect moving in second window
  Eigen::Vector2d acc_mean_w1(0, 0);
  double acc_var_w1 = 0;
  size_t c_idx = imu_queue.size() / 2;

  for (size_t i = c_idx; i < imu_queue.size(); i++) { // new data window mean
    acc_mean_w1 += imu_queue[i]->acc.block<2, 1>(0, 0);
  }

  acc_mean_w1 /= (imu_queue.size() - c_idx);

  for (size_t i = c_idx; i < imu_queue.size(); i++) { // new data window var
    acc_var_w1 += (imu_queue[i]->acc.block<2, 1>(0, 0) - acc_mean_w1)
                      .dot(imu_queue[i]->acc.block<2, 1>(0, 0) - acc_mean_w1);
  }
  acc_var_w1 = sqrt(acc_var_w1 / c_idx);
  logger_->debug("acc_var_w1: {}", acc_var_w1);

  if (acc_var_w1 < acc_var_thresh_) {
    return false;
  }

  // calculate gravity and biases in first window
  Eigen::Vector3d acc_mean_w0(0, 0, 0), gyro_mean_w0(0, 0, 0);
  for (size_t i = 0; i < c_idx; i++) { // old data window
    acc_mean_w0 += imu_queue[i]->acc;
    gyro_mean_w0 += imu_queue[i]->gyro;
  }
  acc_mean_w0 /= c_idx;
  gyro_mean_w0 /= c_idx;

  // rotation from gram schimidt
  Eigen::Matrix3d R_MinI = gram_schmidt(acc_mean_w0);
  Eigen::Matrix3d R_IinM = R_MinI.inverse();
  q_MtoI_ = Eigen::Quaterniond(R_IinM);

  // biases
  Eigen::Vector3d g(0, 0, gravity_);
  bias_acc_ = acc_mean_w0 - R_MinI * g;
  bias_gyro_ = gyro_mean_w0;

  return true;
}

StaticInitializer::~StaticInitializer() {}
