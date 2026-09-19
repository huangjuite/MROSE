#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <spdlog/spdlog.h>

#include <deque>
#include <memory>
#include <vector>

#include "core/params.hpp"
#include "core/rplrio_types.hpp"
#include "factor/VelocityPoseBiasFactor.hpp"
#include "factor/VelocityPoseMeasurementFactor.hpp"
#include "manager/ControledSmoother.hpp"
#include "manager/StaticInitializer.hpp"
#include "velocity/RadarVelocityEstimatorBase.hpp"

typedef VelocityPoseBiasFactor RadarFactor;

namespace rplrio {

class DIOManager {
private:
  rplrio::Params params;
  std::shared_ptr<spdlog::logger> logger_;

  double pre_imu_t = 0;
  int step = 0;
  int imu_count = 0;
  bool correct_now = false;
  bool initialized = false;

  // radar
  std::deque<std::shared_ptr<Radar>> radar_que;

  // imu settings
  std::shared_ptr<gtsam::PreintegrationBase> preintegrated;
  std::deque<std::shared_ptr<IMU>> imu_que;
  std::shared_ptr<IMU> last_imu;
  std::shared_ptr<StaticInitializer> initializer;

  // states
  gtsam::Values initial_values;
  gtsam::NavState prev_state;
  gtsam::NavState prop_state;
  gtsam::imuBias::ConstantBias prev_bias;

  std::vector<std::shared_ptr<Keyframe>> latest_states;

  // optimizer & graph
  gtsam::FixedLagSmoother::KeyTimestampMap key_timestamps;
  std::unique_ptr<ControledSmoother> smoother;
  gtsam::noiseModel::Diagonal::shared_ptr marginalizationPoseModel;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph;

  void graph_init(const Eigen::Quaterniond &rotation,
                  const Eigen::Vector3d &bias_acc,
                  const Eigen::Vector3d &bias_gyro, const double &stamp);
  void update();

  std::shared_ptr<Keyframe> optimize(const Radar &radar, const IMU &imu);

public:
  DIOManager(rplrio::Params params,
             std::shared_ptr<RadarVelocityEstimatorBase> velocity_estimator);
  ~DIOManager();

  void feed_radar(std::shared_ptr<Radar> radar);

  void feed_imu(std::shared_ptr<IMU> imu_data);

  const std::vector<std::shared_ptr<Keyframe>> &get_latest_states() const;

  std::shared_ptr<RadarVelocityEstimatorBase> velocity_estimator;
};

} // namespace rplrio
