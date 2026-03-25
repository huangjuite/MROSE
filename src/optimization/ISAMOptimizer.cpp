#include "optimization/ISAMOptimizer.hpp"
#include "core/RadarPointType.hpp"

#include <gtsam/inference/Symbol.h>

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace rplrio {

template <class PointType>
ISAMOptimizer<PointType>::ISAMOptimizer(
    rplrio::Params params,
    std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator)
    : params_(params), velocity_estimator_(velocity_estimator) {

  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  smoother_ = std::make_unique<ControledSmoother>(params_.lag, isam_params);
}

template <class PointType>
void ISAMOptimizer<PointType>::graph_init(const Eigen::Quaterniond &rotation,
                                          const Eigen::Vector3d &bias_acc,
                                          const Eigen::Vector3d &bias_gyro,
                                          const double &stamp) {
  gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(rotation.w(), rotation.x(),
                                                  rotation.y(), rotation.z()),
                          gtsam::Point3(0, 0, 0));
  gtsam::Vector3 prior_velocity(0, 0, 0);
  gtsam::imuBias::ConstantBias prior_imu_bias(bias_acc, bias_gyro);

  preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      rplrio::imuParams(params_), prior_imu_bias);
  assert(preintegrated);

  initial_values_.insert(X(0), prior_pose);
  initial_values_.insert(V(0), prior_velocity);
  initial_values_.insert(B(0), prior_imu_bias);
  key_timestamps_[X(0)] = stamp;
  key_timestamps_[V(0)] = stamp;
  key_timestamps_[B(0)] = stamp;

  prev_state = gtsam::NavState(prior_pose, prior_velocity);
  prop_state = prev_state;
  prev_bias = prior_imu_bias;

  auto pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  auto velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 1e-3);
  auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  graph_ = std::make_unique<gtsam::NonlinearFactorGraph>();
  graph_->addPrior(X(0), prior_pose, pose_noise_model);
  graph_->addPrior(V(0), prior_velocity, velocity_noise_model);
  graph_->addPrior(B(0), prior_imu_bias, bias_noise_model);

  marginalization_pose_model_ =
      gtsam::noiseModel::Diagonal::Sigmas(params_.marginalization_sigma);
}

template <class PointType>
std::shared_ptr<rplrio::Keyframe<PointType>>
ISAMOptimizer<PointType>::optimize(const rplrio::Radar<PointType> &radar,
                                   const rplrio::IMU &imu) {
  double current_t = radar.stamp;

  // IMU factor
  if (preintegrated->deltaTij() > 1e-6) {
    auto preint_imu_combined =
        dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(
            *preintegrated);
    gtsam::CombinedImuFactor imu_factor(X(step - 1), V(step - 1), X(step),
                                        V(step), B(step - 1), B(step),
                                        preint_imu_combined);
    graph_->add(imu_factor);

    prop_state = preintegrated->predict(prev_state, prev_bias);
    initial_values_.insert(X(step), prop_state.pose());
    initial_values_.insert(V(step), prop_state.v());
    initial_values_.insert(B(step), prev_bias);

    key_timestamps_[X(step)] = current_t;
    key_timestamps_[V(step)] = current_t;
    key_timestamps_[B(step)] = current_t;
  }

  // Radar velocity factor
  gtsam::Vector3 measured_v;
  gtsam::Matrix33 radar_velocity_cov;
  gtsam::noiseModel::Base::shared_ptr rv_noise;
  typename pcl::PointCloud<PointType>::Ptr radar_inlier;

  if (radar.pc) {
    velocity_estimator_->set_initial_velocity(radar.param.R_r_i.transpose() *
                                              prop_state.bodyVelocity());
    velocity_estimator_->set_doppler_sigma(radar.param.doppler_std);
    measured_v = velocity_estimator_->optimize(radar.pc);
    radar_inlier = velocity_estimator_->get_inlier(radar.pc, measured_v);
    pcl::transformPointCloud(*radar_inlier, *radar_inlier, radar.param.T_r_i);
    radar_velocity_cov =
        velocity_estimator_->get_covariance(radar.pc, measured_v);
  } else {
    measured_v = radar.vel;
    radar_velocity_cov = radar.vel_cov;
  }

  measured_v = radar.param.R_r_i * measured_v;
  radar_velocity_cov =
      radar.param.R_r_i * radar_velocity_cov * radar.param.R_r_i.transpose();

  if (params_.bv_use_huber) {
    rv_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params_.bv_huber_k),
        gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov));
  } else {
    rv_noise = gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov);
  }

  RadarFactor rfac(V(step), X(step), B(step), measured_v, imu.gyro,
                   radar.param.T_r_i, rv_noise);
  graph_->add(rfac);

  // Optimize
  smoother_->update(*graph_, initial_values_, key_timestamps_,
                    marginalization_pose_model_);
  graph_->resize(0);
  initial_values_.clear();
  key_timestamps_.clear();

  // Update state
  auto opt_x = smoother_->calculateEstimate<gtsam::Pose3>(X(step));
  auto opt_v = smoother_->calculateEstimate<gtsam::Vector3>(V(step));
  prev_state = gtsam::NavState(opt_x, opt_v);
  prev_bias =
      smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(step));
  preintegrated->resetIntegrationAndSetBias(prev_bias);

  gtsam::Matrix x_cov = smoother_->marginalCovariance(X(step));
  gtsam::Matrix v_cov = smoother_->marginalCovariance(V(step));
  auto v_err = rfac.evaluateError(opt_v, opt_x, prev_bias, {}, {}, {});

  return std::make_shared<rplrio::Keyframe<PointType>>(
      rplrio::Keyframe<PointType>{.stamp = current_t,
                                  .seq = static_cast<uint>(step),
                                  .name = radar.frame_id,
                                  .state = prev_state,
                                  .pose_cov = x_cov,
                                  .velocity_cov = v_cov,
                                  .v_err = v_err,
                                  .imu = imu,
                                  .imu_bias = prev_bias,
                                  .radar_points = radar_inlier,
                                  .radar_velocity = measured_v,
                                  .radar_velocity_cov = radar_velocity_cov});
}

template class ISAMOptimizer<pcl::PointXYZD>;
template class ISAMOptimizer<pcl::PointXYZSD>;
template class ISAMOptimizer<pcl::PointXYZSDW>;
template class ISAMOptimizer<pcl::PointHugin>;

} // namespace rplrio
