#pragma once

#include "VelocityPoseBiasFactor.h"
#include "form/optimization/constraints.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace rplrio {

/// Extends form::ConstraintManager with velocity (V) and bias (B) states,
/// IMU preintegration factors, and body velocity (radar) factors.
class RIOConstraintManager : public form::ConstraintManager {
public:
  RIOConstraintManager() : form::ConstraintManager() {}
  RIOConstraintManager(const form::ConstraintManager::Params &params)
      : form::ConstraintManager(params), m_opt_params(params.opt_params) {}

  // ----------------------------- Doers ----------------------------- //

  /// Step the manager: inserts X(scan) via base, plus V(scan) and B(scan).
  /// Returns the new scan index. Use get_current_constraints() for the map.
  size_t step(const gtsam::Pose3 &pose, const gtsam::Vector3 &velocity,
              const gtsam::imuBias::ConstantBias &bias) noexcept;

  /// Add a preintegrated IMU factor between consecutive scans.
  void add_imu_factor(const gtsam::CombinedImuFactor &factor) noexcept;

  /// Add a body velocity (radar Doppler) factor for the current scan.
  void add_body_velocity_factor(const VelocityPoseBiasFactor &factor) noexcept;

  /// Optimize over feature, IMU, and body velocity factors.
  gtsam::Values optimize(bool fast = false) noexcept;

  /// Marginalize the given scans (X, V, B) and associated factors.
  void marginalize(const std::vector<form::ScanIndex> &scans) noexcept;

  /// Update all values: X keys go to base, V and B keys stay in derived storage.
  void update_values(const gtsam::Values &values) noexcept;

  // ----------------------------- Getters ----------------------------- //

  gtsam::Vector3 get_velocity(form::ScanIndex scan) const noexcept;
  gtsam::imuBias::ConstantBias get_bias(form::ScanIndex scan) const noexcept;

  /// All values: X (from base) merged with V and B.
  gtsam::Values get_all_values() const noexcept;

private:
  /// Velocity and bias values V(i), B(i).
  gtsam::Values m_imu_values;

  /// IMU preintegration factors between consecutive scans.
  gtsam::NonlinearFactorGraph m_imu_factors;

  /// Body velocity (radar Doppler) factors per scan.
  gtsam::NonlinearFactorGraph m_bv_factors;

  gtsam::LevenbergMarquardtParams m_opt_params;
};

} // namespace rplrio
