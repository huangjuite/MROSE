#include "DIOManager.h"
#include "RadarPointType.h"
#include <spdlog/sinks/stdout_color_sinks.h>

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace rplrio {

template <class PointType>
DIOManager<PointType>::DIOManager(
    rplrio::Params params,
    std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator) {

  logger_ = spdlog::stdout_color_mt("rplrio");
  logger_->set_pattern("[%^%l%$] %v");
  logger_->set_level(static_cast<spdlog::level::level_enum>(params.log_level));

  this->params = params;
  this->velocity_estimator = velocity_estimator;

  velocity_estimator->set_inlier_thresh(params.doppler_inlier_thresh);

  // optimizer init
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.01;
  parameters.relinearizeSkip = 1;
  smootherISAM2 = std::make_unique<ControledSmoother>(params.lag, parameters);

  // imu init
  if (params.use_static_init) {
    initialized = false;
    initializer = std::make_unique<StaticInitializer>(
        params.init_window, params.init_thres, params.gravity);
  }
}

template <class PointType>
void DIOManager<PointType>::graph_init(const Eigen::Quaterniond &rotation,
                                       const Eigen::Vector3d &bias_acc,
                                       const Eigen::Vector3d &bias_gyro,
                                       const double &stamp) {

  // priors
  gtsam::Pose3 prior_pose(gtsam::Rot3::Quaternion(rotation.w(), rotation.x(),
                                                  rotation.y(), rotation.z()),
                          gtsam::Point3(0, 0, 0));
  gtsam::Vector3 prior_velocity(0, 0, 0);
  gtsam::imuBias::ConstantBias prior_imu_bias(bias_acc, bias_gyro);
  preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(
      rplrio::imuParams(params), prior_imu_bias);
  assert(preintegrated);

  // pose init
  initial_values.insert(X(0), prior_pose);
  initial_values.insert(V(0), prior_velocity);
  initial_values.insert(B(0), prior_imu_bias);
  key_timestamps[X(0)] = stamp;
  key_timestamps[V(0)] = stamp;
  key_timestamps[B(0)] = stamp;

  prev_state = gtsam::NavState(prior_pose, prior_velocity);
  prop_state = prev_state;
  prev_bias = prior_imu_bias;

  // prior noise model
  auto pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3)
          .finished()); // rad,rad,rad,m, m, m
  auto velocity_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(3, 1e-3); // m/s
  auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

  // Add all prior
  graph = std::make_unique<gtsam::NonlinearFactorGraph>();
  graph->addPrior(X(0), prior_pose, pose_noise_model);
  graph->addPrior(V(0), prior_velocity, velocity_noise_model);
  graph->addPrior(B(0), prior_imu_bias, bias_noise_model);

  // set marginlization model
  marginalizationPoseModel = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 10, 10, 10, 10, 10, 10).finished());

  initialized = true;
}

template <class PointType> void DIOManager<PointType>::update() {
  // remove out of date radar measurements
  while (!radar_que.empty() &&
         radar_que.front()->stamp < imu_que.front()->stamp) {
    radar_que.pop_front();
  }

  if (radar_que.empty() || radar_que.front()->stamp > imu_que.back()->stamp) {
    return;
  }

  auto radar_msg = radar_que.front();
  double radar_stamp = radar_msg->stamp;
  radar_que.pop_front();

  // integrate imu measurements
  std::shared_ptr<rplrio::IMU> last_imu;
  size_t imu_size = 0;
  logger_->debug("------------------------------");
  logger_->debug("key frame {}", radar_msg->param.topic);
  logger_->debug("stamp: {:.6f}", radar_stamp);
  logger_->debug("imu queue size: {}", imu_que.size());

  while (!imu_que.empty() && radar_stamp > imu_que.front()->stamp) {
    auto imu_data = imu_que.front();
    double dt = imu_data->stamp - pre_imu_t;
    preintegrated->integrateMeasurement(imu_data->acc, imu_data->gyro, dt);
    pre_imu_t = imu_data->stamp;

    last_imu = imu_data;
    imu_que.pop_front();
    imu_size++;
  }
  logger_->debug("imu integrated: {}", imu_size);

  auto next_imu = imu_que.front();
  rplrio::IMU interp;
  bool interp_success =
      rplrio::IMU::interpolate(interp, radar_stamp, *last_imu, *next_imu);
  if (interp_success) {
    preintegrated->integrateMeasurement(interp.acc, interp.gyro,
                                        interp.stamp - pre_imu_t);
    pre_imu_t = interp.stamp;
  }

  step++;
  auto keyframe = optimize(*radar_msg);
  latest_states.emplace_back(keyframe);

  // recurisvely update until latest radar data is processed
  update();
}

template <class PointType>
std::shared_ptr<rplrio::Keyframe<PointType>>
DIOManager<PointType>::optimize(const DIORadar &radar) {
  double current_t = radar.stamp;

  // combined IMU factor
  // ---------------------------------------------------------------------------
  auto preint_imu_combined =
      dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(
          *preintegrated);
  gtsam::CombinedImuFactor imu_factor(X(step - 1), V(step - 1), X(step),
                                      V(step), B(step - 1), B(step),
                                      preint_imu_combined);
  graph->add(imu_factor);

  prop_state = preintegrated->predict(prev_state, prev_bias);
  initial_values.insert(X(step), prop_state.pose());
  initial_values.insert(V(step), prop_state.v());
  initial_values.insert(B(step), prev_bias);

  key_timestamps[X(step)] = current_t;
  key_timestamps[V(step)] = current_t;
  key_timestamps[B(step)] = current_t;

  // ------------------------------------------------------------------------------------
  gtsam::Vector3 measured_v;
  gtsam::noiseModel::Base::shared_ptr rv_noise;

  velocity_estimator->set_initial_velocity(radar.param.R_r_i.transpose() *
                                           prop_state.bodyVelocity());
  measured_v = velocity_estimator->optimize(radar.pc);
  auto radar_inlier = velocity_estimator->get_inlier(radar.pc, measured_v);

  gtsam::Matrix radar_velocity_cov =
      velocity_estimator->get_covariance(radar.pc, measured_v);

  // velocity pose measurement
  measured_v = radar.param.R_r_i * measured_v;
  radar_velocity_cov =
      radar.param.R_r_i * radar_velocity_cov * radar.param.R_r_i.transpose();
  pcl::transformPointCloud(*radar_inlier, *radar_inlier, radar.param.T_r_i);

  if (params.bv_use_huber) {
    rv_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.bv_huber_k),
        gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov));
  } else {
    rv_noise = gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov);
  }

  graph->add(RadarFactor(V(step), X(step), measured_v, rv_noise));

  // optimize
  // ----------------------------------------------------------------------------
  smootherISAM2->update(*graph, initial_values, key_timestamps,
                        marginalizationPoseModel);

  graph->resize(0);
  initial_values.clear();
  key_timestamps.clear();

  //-----------------------------------------------------------------------------
  // Overwrite the beginning of the preintegration for the next step.
  prev_state = gtsam::NavState(
      smootherISAM2->calculateEstimate<gtsam::Pose3>(X(step)),
      smootherISAM2->calculateEstimate<gtsam::Vector3>(V(step)));
  prev_bias =
      smootherISAM2->calculateEstimate<gtsam::imuBias::ConstantBias>(B(step));
  preintegrated->resetIntegrationAndSetBias(prev_bias);

  gtsam::Matrix x_cov = smootherISAM2->marginalCovariance(X(step));
  gtsam::Matrix v_cov = smootherISAM2->marginalCovariance(V(step));

  // key frame informations
  // -----------------------------------------------------------------------------
  auto key_frame = std::make_shared<DIOKeyframe>();
  key_frame->stamp = current_t;
  key_frame->seq = step;
  key_frame->state = prev_state;
  key_frame->imu_bias = prev_bias;
  key_frame->pose_cov = x_cov;
  key_frame->velocity_cov = v_cov;
  key_frame->radar_points = radar_inlier;
  key_frame->radar_velocity = measured_v;
  key_frame->radar_velocity_cov = radar_velocity_cov;

  return key_frame;
}

template <class PointType>
void DIOManager<PointType>::feed_imu(std::shared_ptr<rplrio::IMU> imu_data) {

  if (params.use_static_init && !initialized) {
    bool init_success = initializer->init_update(imu_data);
    if (init_success) {

      Eigen::Vector3d bias_a = initializer->get_bias_acc();
      Eigen::Vector3d bias_g = initializer->get_bias_gyro();
      Eigen::Quaterniond init_r = initializer->get_rotation();

      pre_imu_t = initializer->get_first_stamp();
      graph_init(init_r, bias_a, bias_g, pre_imu_t);
      imu_que = initializer->get_imu_queue();
      logger_->info("static init");
      logger_->info("bias acc: {}, {}, {}", bias_a[0], bias_a[1], bias_a[2]);
      logger_->info("bias gyro: {}, {}, {}", bias_g[0], bias_g[1], bias_g[2]);
      auto rot_mat = init_r.toRotationMatrix();
      logger_->info("rotation:");
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(0, 0),
                    rot_mat(0, 1), rot_mat(0, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(1, 0),
                    rot_mat(1, 1), rot_mat(1, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(2, 0),
                    rot_mat(2, 1), rot_mat(2, 2));

      update();
    }
  } else if (!params.use_static_init && !initialized) {

    Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
    Eigen::Quaterniond init_r = Eigen::Quaterniond::Identity();

    pre_imu_t = imu_data->stamp;
    graph_init(init_r, bias_a, bias_g, pre_imu_t);

  } else {
    imu_que.emplace_back(imu_data);
  }
}

template <class PointType>
void DIOManager<PointType>::feed_radar(std::shared_ptr<DIORadar> radar) {

  radar_que.emplace_back(radar);

  if (!initialized) {
    return;
  }

  latest_states.clear();

  update();

  return;
}

template <class PointType>
const std::vector<std::shared_ptr<rplrio::Keyframe<PointType>>> &
DIOManager<PointType>::get_latest_states() const {
  return latest_states;
}

template <class PointType> DIOManager<PointType>::~DIOManager() {}

template class DIOManager<pcl::PointXYZD>;
template class DIOManager<pcl::PointXYZSD>;
template class DIOManager<pcl::PointXYZSDW>;
template class DIOManager<pcl::PointHugin>;

} // namespace rplrio