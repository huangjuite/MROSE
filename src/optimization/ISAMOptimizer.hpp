#pragma once

#include <memory>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "optimization/ControledSmoother.hpp"
#include "radar/RadarVelocityEstimatorBase.hpp"
#include "factors/VelocityPoseBiasFactor.hpp"
#include "core/params.hpp"
#include "core/rplrio_types.hpp"

typedef VelocityPoseBiasFactor RadarFactor;

namespace rplrio {

template <class PointType>
class ISAMOptimizer {
public:
  // State shared with manager (integration and propagation)
  std::shared_ptr<gtsam::PreintegrationType> preintegrated;
  gtsam::NavState prev_state;
  gtsam::NavState prop_state;
  gtsam::imuBias::ConstantBias prev_bias;
  int step = 0;

  ISAMOptimizer(
      rplrio::Params params,
      std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator);

  void graph_init(const Eigen::Quaterniond &rotation,
                  const Eigen::Vector3d &bias_acc,
                  const Eigen::Vector3d &bias_gyro, const double &stamp);

  std::shared_ptr<rplrio::Keyframe<PointType>>
  optimize(const rplrio::Radar<PointType> &radar, const rplrio::IMU &imu);

private:
  rplrio::Params params_;
  std::shared_ptr<RadarVelocityEstimatorBase<PointType>> velocity_estimator_;

  std::unique_ptr<ControledSmoother> smoother_;
  std::unique_ptr<gtsam::NonlinearFactorGraph> graph_;
  gtsam::Values initial_values_;
  gtsam::FixedLagSmoother::KeyTimestampMap key_timestamps_;
  gtsam::noiseModel::Diagonal::shared_ptr marginalization_pose_model_;
};

} // namespace rplrio
