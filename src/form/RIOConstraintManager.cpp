#include "form/RIOConstraintManager.hpp"
#include "form/optimization/gtsam.hpp"

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace rplrio {

size_t RIOConstraintManager::step(
    const gtsam::Pose3 &pose, const gtsam::Vector3 &velocity,
    const gtsam::imuBias::ConstantBias &bias) noexcept {
  const auto scan_idx = std::get<0>(form::ConstraintManager::step(pose));

  m_imu_values.insert(V(scan_idx), velocity);
  m_imu_values.insert(B(scan_idx), bias);

  return scan_idx;
}

void RIOConstraintManager::add_imu_factor(
    const gtsam::CombinedImuFactor &factor) noexcept {
  m_imu_factors.push_back(factor.clone());
}

void RIOConstraintManager::add_body_velocity_factor(
    const VelocityPoseBiasFactor &factor) noexcept {
  m_bv_factors.push_back(factor.clone());
}

gtsam::Values RIOConstraintManager::optimize(bool fast) noexcept {
  auto graph = get_graph(fast);
  // gtsam::NonlinearFactorGraph graph;
  graph.push_back(m_imu_factors);
  graph.push_back(m_bv_factors);

  form::DenseLMOptimizer optimizer(graph, get_all_values(), m_opt_params);
  return optimizer.optimize();
}

void RIOConstraintManager::marginalize(
    const std::vector<form::ScanIndex> &scans) noexcept {
  // Drop IMU and BV factors involving any marginalized X/V/B key.
  auto involves_scan = [&](const gtsam::NonlinearFactor::shared_ptr &f) {
    for (const auto &key : f->keys()) {
      const auto sym = gtsam::Symbol(key);
      for (const auto &s : scans) {
        if (sym.index() == s &&
            (sym.chr() == 'x' || sym.chr() == 'v' || sym.chr() == 'b')) {
          return true;
        }
      }
    }
    return false;
  };

  gtsam::NonlinearFactorGraph kept_imu, kept_bv;
  for (const auto &f : m_imu_factors)
    if (f && !involves_scan(f))
      kept_imu.push_back(f);
  for (const auto &f : m_bv_factors)
    if (f && !involves_scan(f))
      kept_bv.push_back(f);
  m_imu_factors = std::move(kept_imu);
  m_bv_factors = std::move(kept_bv);

  // Remove V(i) and B(i) for marginalized scans.
  for (const auto &s : scans) {
    if (m_imu_values.exists(V(s)))
      m_imu_values.erase(V(s));
    if (m_imu_values.exists(B(s)))
      m_imu_values.erase(B(s));
  }
  std::cout << "After marginalization, " << m_imu_factors.size() << " IMU factors and "
            << m_bv_factors.size() << " BV factors remain." << std::endl;

  // Delegate X and feature factor marginalization to the base.
  form::ConstraintManager::marginalize(scans);
}

void RIOConstraintManager::update_values(
    const gtsam::Values &values) noexcept {
  gtsam::Values pose_values, imu_values;
  for (const auto &kv : values) {
    const auto sym = gtsam::Symbol(kv.key);
    if (sym.chr() == 'x')
      pose_values.insert(kv.key, kv.value);
    else
      imu_values.insert(kv.key, kv.value);
  }
  if (!pose_values.empty())
    form::ConstraintManager::update_values(pose_values);
  m_imu_values.update(imu_values);
}

gtsam::Values RIOConstraintManager::get_all_values() const noexcept {
  gtsam::Values all = get_values(); // X(i) from base
  all.insert(m_imu_values);         // V(i), B(i)
  return all;
}

gtsam::Vector3
RIOConstraintManager::get_velocity(form::ScanIndex scan) const noexcept {
  return m_imu_values.at<gtsam::Vector3>(V(scan));
}

gtsam::imuBias::ConstantBias
RIOConstraintManager::get_bias(form::ScanIndex scan) const noexcept {
  return m_imu_values.at<gtsam::imuBias::ConstantBias>(B(scan));
}

} // namespace rplrio
