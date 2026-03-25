#include "manager/RIOManager.hpp"
#include "core/RadarPointType.hpp"
#include <gtsam/inference/Symbol.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

namespace rplrio {

template <class PointType>
RIOManager<PointType>::RIOManager(
    rplrio::Params params,
    std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator,
    RadarEstimator::Params form_params)
    : form_params(form_params), m_extractor(form_params.extraction),
      m_constraints(form_params.constraints),
      m_matcher{form::Matcher<form::PlanarFeat>(form_params.matcher, 0),
                form::Matcher<form::PointFeat>(form_params.matcher, 0)},
      m_keyscanner(form_params.scans),
      m_keypoint_map{form::KeypointMap<form::PlanarFeat>(form_params.map),
                     form::KeypointMap<form::PointFeat>(form_params.map)} {

  logger_ = spdlog::stdout_color_mt("rplrio");
  logger_->set_pattern("[%^%l%$] %v");
  logger_->set_level(static_cast<spdlog::level::level_enum>(params.log_level));

  this->params = params;
  this->velocity_estimator = velocity_estimator;

  velocity_estimator->set_inlier_thresh(params.doppler_inlier_thresh);

  // imu init
  if (params.use_static_init) {
    initialized = false;
    initializer = std::make_unique<StaticInitializer>(
        params.init_window, params.init_thres, params.gravity);
  }
}

template <class PointType>
void RIOManager<PointType>::graph_init(const Eigen::Quaterniond &rotation,
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

  prev_state = gtsam::NavState(prior_pose, prior_velocity);
  prop_state = prev_state;
  prev_bias = prior_imu_bias;

  // Initialize constraint manager with step 0
  m_constraints.step(prior_pose, prior_velocity, prior_imu_bias);

  initialized = true;
}

template <class PointType> void RIOManager<PointType>::update() {
  // remove out of date radar measurements
  while (!radar_que.empty() && radar_que.front()->stamp < pre_imu_t - 1e-6) {
    logger_->debug("out of date: {}, {:.6f}", radar_que.front()->frame_id,
                   radar_que.front()->stamp);
    logger_->debug("imu front {:.6f}, pre: {:.6f}", imu_que.front()->stamp,
                   pre_imu_t);
    radar_que.pop_front();
  }

  // IMU did not catch up yet
  if (radar_que.empty() || radar_que.front()->stamp > imu_que.back()->stamp) {
    return;
  }

  auto radar_msg = radar_que.front();
  double radar_stamp = radar_msg->stamp;
  radar_que.pop_front();

  // integrate imu measurements
  size_t imu_size = 0;
  while (!imu_que.empty() && (imu_que.front()->stamp - radar_stamp) < 1e-6) {
    auto imu_data = imu_que.front();
    double dt = imu_data->stamp - pre_imu_t;
    preintegrated->integrateMeasurement(imu_data->acc, imu_data->gyro, dt);
    pre_imu_t = imu_data->stamp;
    last_imu = imu_data;
    imu_que.pop_front();
    imu_size++;
  }

  logger_->info("------------------------------");
  logger_->info("key frame {}", radar_msg->frame_id);
  logger_->info("imu queue size: {}", imu_que.size());
  logger_->info("radar queue size: {}", radar_que.size());
  logger_->info("imu integrated: {}", imu_size);

  // close to previous key frame
  if (imu_size == 0) {
    logger_->debug("only add radar.");
    auto keyframe = optimize(*radar_msg, *last_imu);
    latest_states.emplace_back(keyframe);
  } else { // new key frame

    auto next_imu = imu_que.front();
    rplrio::IMU interp;

    bool interp_success =
        rplrio::IMU::interpolate(interp, radar_stamp, *last_imu, *next_imu);
    if (interp_success) {
      preintegrated->integrateMeasurement(interp.acc, interp.gyro,
                                          interp.stamp - pre_imu_t);
      pre_imu_t = interp.stamp;
      *last_imu = interp;
    }

    auto keyframe = optimize(*radar_msg, *last_imu);
    latest_states.emplace_back(keyframe);
  }

  logger_->info("time: {:.6f}", radar_stamp);

  // recurisvely update until latest radar data is processed
  update();
}

template <class PointType>
std::shared_ptr<rplrio::Keyframe<PointType>>
RIOManager<PointType>::optimize(const RIORadar &radar, const rplrio::IMU &imu) {
  double current_t = radar.stamp;

  // combined IMU factor
  // ---------------------------------------------------------------------------
  if (preintegrated->deltaTij() > 1e-6) {
    prop_state = preintegrated->predict(prev_state, prev_bias);
  }

  // ------------------------------------------------------------------------------------
  gtsam::Vector3 measured_v;
  gtsam::Matrix33 radar_velocity_cov;
  gtsam::noiseModel::Base::shared_ptr rv_noise;
  typename pcl::PointCloud<PointType>::Ptr radar_inlier;

  velocity_estimator->set_initial_velocity(radar.param.R_r_i.transpose() *
                                           prop_state.bodyVelocity());
  velocity_estimator->set_doppler_sigma(radar.param.doppler_std);
  measured_v = velocity_estimator->optimize(radar.pc);
  radar_inlier = velocity_estimator->get_inlier(radar.pc, measured_v);
  pcl::transformPointCloud(*radar_inlier, *radar_inlier, radar.param.T_r_i);
  radar_velocity_cov = velocity_estimator->get_covariance(radar.pc, measured_v);

  // rotate to IMU frame
  measured_v = radar.param.R_r_i * measured_v;
  radar_velocity_cov =
      radar.param.R_r_i * radar_velocity_cov * radar.param.R_r_i.transpose();

  // noise model
  if (params.bv_use_huber) {
    rv_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(params.bv_huber_k),
        gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov));
  } else {
    rv_noise = gtsam::noiseModel::Gaussian::Covariance(radar_velocity_cov);
  }

  // ----------------------------------------------------------------------------
  // form: register scan for scan-to-map matching

  // Step constraint manager with propagated IMU state
  const auto scan_idx =
      m_constraints.step(prop_state.pose(), prop_state.v(), prev_bias);
  auto &scan_constraints = m_constraints.get_current_constraints();

  // Add IMU factor now that we have consecutive scan indices
  if (preintegrated->deltaTij() > 1e-6) {
    auto preint_imu_combined =
        dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(
            *preintegrated);
    gtsam::CombinedImuFactor imu_factor(X(scan_idx - 1), V(scan_idx - 1),
                                        X(scan_idx), V(scan_idx), B(scan_idx - 1),
                                        B(scan_idx), preint_imu_combined);
    m_constraints.add_imu_factor(imu_factor);
  }

  // Add body velocity factor
  RadarFactor rfac(V(scan_idx), X(scan_idx), B(scan_idx), measured_v, imu.gyro,
                   radar.param.T_r_i, rv_noise);
  m_constraints.add_body_velocity_factor(rfac);

  // ----------------------------------------------------------------------------
  // Extract Features
  const auto keypoints = m_extractor.extract<PointType>(radar_inlier, scan_idx);
  latest_keypoints = keypoints;
  const auto num_keypoints =
      std::get<0>(keypoints).size() + std::get<1>(keypoints).size();
  logger_->info("extracted {} planar and {} point features",
                std::get<0>(keypoints).size(), std::get<1>(keypoints).size());

  // Generate World Map
  auto planar_map = std::get<0>(m_keypoint_map)
                        .to_voxel_map(m_constraints.get_values(),
                                      form_params.matcher.max_dist_matching);
  // auto point_map = std::get<1>(m_keypoint_map)
  //                      .to_voxel_map(m_constraints.get_values(),
  //                                    form_params.matcher.max_dist_matching);

  // ICP loop
  gtsam::Values new_values;
  auto estimates = [&](size_t i) { return m_constraints.get_pose(i); };
  size_t icp_steps = 0;
  for (size_t idx = 0; idx < form_params.matcher.max_num_rematches; ++idx) {
    auto before = m_constraints.get_current_pose();

    std::get<0>(m_matcher).template match<0>(planar_map, std::get<0>(keypoints),
                                             estimates, scan_constraints);
    // std::get<1>(m_matcher).template match<1>(point_map, std::get<1>(keypoints),
    //                                          estimates, scan_constraints);

    new_values = m_constraints.optimize(true);
    const auto after = new_values.at<gtsam::Pose3>(X(scan_idx));
    const auto diff = before.localCoordinates(after).norm();
    icp_steps = idx + 1;
    if (diff < form_params.matcher.new_pose_threshold) {
      break;
    }
    m_constraints.update_current_pose(after);
  }
  logger_->info("ICP steps: {}", icp_steps);

  // Full Nonlinear Optimization (feature + IMU + body velocity factors)
  new_values = m_constraints.optimize(false);
  m_constraints.update_values(new_values);

  // Map insertions
  std::get<0>(m_keypoint_map).insert_matches(std::get<0>(m_matcher).get_matches());
  // std::get<1>(m_keypoint_map).insert_matches(std::get<1>(m_matcher).get_matches());

  // Keyscan selection
  const auto connections = [&](form::ScanIndex i) {
    return m_constraints.num_recent_connections(i, m_keyscanner.oldest_rf());
  };
  auto marg_scans = m_keyscanner.step(scan_idx, num_keypoints, connections);

  // Marginalization
  {
    std::string indices;
    for (const auto &idx : marg_scans) {
      indices += std::to_string(idx) + " ";
    }
    logger_->info("marginalizing {} scans: [{}]", marg_scans.size(), indices);
  }
  m_constraints.marginalize(marg_scans);
  std::get<0>(m_keypoint_map).remove(marg_scans);
  // std::get<1>(m_keypoint_map).remove(marg_scans);

  //-----------------------------------------------------------------------------
  // Extract optimized state and reset preintegration for next step.
  auto opt_x = new_values.at<gtsam::Pose3>(X(scan_idx));
  auto opt_v = new_values.at<gtsam::Vector3>(V(scan_idx));

  prev_state = gtsam::NavState(opt_x, opt_v);
  prev_bias = new_values.at<gtsam::imuBias::ConstantBias>(B(scan_idx));
  preintegrated->resetIntegrationAndSetBias(prev_bias);

  // TODO: compute velocity covariance from the optimization result
  gtsam::Matrix x_cov = gtsam::Matrix::Zero(6, 6);
  gtsam::Matrix v_cov = gtsam::Matrix::Zero(3, 3);
  auto v_err = rfac.evaluateError(opt_v, opt_x, prev_bias, {}, {}, {});

  // key frame informations
  // -----------------------------------------------------------------------------
  auto key_frame = std::make_shared<RIOKeyframe>(
      RIOKeyframe{.stamp = current_t,
                  .seq = static_cast<uint>(scan_idx),
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

  return key_frame;
}

template <class PointType>
void RIOManager<PointType>::feed_imu(std::shared_ptr<rplrio::IMU> imu_data) {

  if (params.use_static_init && !initialized) {
    bool init_success = initializer->init_update(imu_data);
    if (init_success) {

      Eigen::Vector3d bias_a = initializer->get_bias_acc();
      Eigen::Vector3d bias_g = initializer->get_bias_gyro();
      Eigen::Quaterniond init_r = initializer->get_rotation();

      pre_imu_t = initializer->get_first_stamp();
      graph_init(init_r, bias_a, bias_g, pre_imu_t);
      imu_que = initializer->get_imu_queue();
      last_imu = imu_que.front();
      auto rot_mat = init_r.toRotationMatrix();

      logger_->info("static init");
      logger_->info("bias acc: {}, {}, {}", bias_a[0], bias_a[1], bias_a[2]);
      logger_->info("bias gyro: {}, {}, {}", bias_g[0], bias_g[1], bias_g[2]);
      logger_->info("rotation:");
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(0, 0), rot_mat(0, 1),
                    rot_mat(0, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(1, 0), rot_mat(1, 1),
                    rot_mat(1, 2));
      logger_->info("  [{:9.6f}, {:9.6f}, {:9.6f}]", rot_mat(2, 0), rot_mat(2, 1),
                    rot_mat(2, 2));

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
void RIOManager<PointType>::feed_radar(std::shared_ptr<RIORadar> radar) {

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
RIOManager<PointType>::get_latest_states() const {
  return latest_states;
}

template <class PointType> RIOManager<PointType>::~RIOManager() {}

template class RIOManager<pcl::PointXYZD>;
template class RIOManager<pcl::PointXYZSD>;
template class RIOManager<pcl::PointXYZSDW>;
template class RIOManager<pcl::PointHugin>;

} // namespace rplrio
