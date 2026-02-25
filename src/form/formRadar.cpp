#define PCL_NO_PRECOMPILE
#include "form/formRadar.h"
#include "form/utils.hpp"
#include "RadarPointType.h"

namespace rplrio {

using gtsam::Pose3;
using gtsam::Velocity3;
using gtsam::symbol_shorthand::X;
using form::PlanarFeat;
using form::PointFeat;
using form::ScanIndex;

RadarEstimator::RadarEstimator(const RadarEstimator::Params &params) noexcept
    : m_params(params), m_extractor(params.extraction),
      m_constraints(params.constraints),
      m_matcher{form::Matcher<PlanarFeat>(params.matcher, 0),
                form::Matcher<PointFeat>(params.matcher, 0)},
      m_keyscanner(params.scans),
      m_keypoint_map{form::KeypointMap<PlanarFeat>(m_params.map),
                     form::KeypointMap<PointFeat>(m_params.map)} {}

template <typename PointT>
std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
RadarEstimator::register_scan(
    const typename pcl::PointCloud<PointT>::Ptr &cloud) noexcept {
  constexpr auto SEQ = std::make_index_sequence<2>{};

  //
  // ############################ Feature Extraction ############################ //
  //
  // ------------------------------ Initialization ------------------------------ //
  Pose3 prediction = m_constraints.predict_next();
  auto [scan_idx, scan_constraints] = m_constraints.step(prediction);

  // ----------------------------- Extract Features ----------------------------- //
  const auto keypoints = m_extractor.extract<PointT>(cloud, scan_idx);
  const auto num_keypoints =
      std::apply([](auto &...kps) { return (kps.size() + ...); }, keypoints);

  //
  // ############################### Optimization ############################### //
  //
  // ---------------------------- Generate World Map ---------------------------- //
  const auto world_map = form::tuple::transform(m_keypoint_map, [&](auto &map) {
    return map.to_voxel_map(m_constraints.get_values(),
                            m_params.matcher.max_dist_matching);
  });

  // ICP loop
  gtsam::Values new_values;
  auto estimates = [&](size_t i) { return m_constraints.get_pose(i); };
  for (size_t idx = 0; idx < m_params.matcher.max_num_rematches; ++idx) {
    auto before = m_constraints.get_current_pose();

    // -------------------------------- Matching -------------------------------- //
    form::tuple::for_seq(SEQ, [&](auto I) {
      std::get<I>(m_matcher).template match<I>(std::get<I>(world_map),
                                               std::get<I>(keypoints), estimates,
                                               scan_constraints);
    });

    // ---------------------- Semi-Linearized Optimization ---------------------- //
    new_values = m_constraints.optimize(true);
    const auto after = new_values.at<Pose3>(X(scan_idx));
    const auto diff = before.localCoordinates(after).norm();
    if (diff < m_params.matcher.new_pose_threshold) {
      break;
    }
    m_constraints.update_current_pose(after);
  }

  // ------------------------ Full Nonlinear Optimization ------------------------ //
  new_values = m_constraints.optimize(false);
  m_constraints.update_values(new_values);

  //
  // ################################## Mapping ################################## //
  //
  // ------------------------------ Map insertions ------------------------------ //
  form::tuple::for_seq(SEQ, [&](auto I) {
    std::get<I>(m_keypoint_map).insert_matches(std::get<I>(m_matcher).get_matches());
  });

  // ---------------------------- Keyscan Selection ---------------------------- //
  const auto connections = [&](ScanIndex i) {
    return m_constraints.num_recent_connections(i, m_keyscanner.oldest_rf());
  };
  auto marg_scans = m_keyscanner.step(scan_idx, num_keypoints, connections);

  // ----------------------------- Marginalization ----------------------------- //
  m_constraints.marginalize(marg_scans);
  form::tuple::for_each(m_keypoint_map, [&](auto &map) { map.remove(marg_scans); });

  return keypoints;
}

// Explicit instantiations
template std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
RadarEstimator::register_scan<pcl::PointXYZD>(
    const pcl::PointCloud<pcl::PointXYZD>::Ptr &) noexcept;

template std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
RadarEstimator::register_scan<pcl::PointXYZSD>(
    const pcl::PointCloud<pcl::PointXYZSD>::Ptr &) noexcept;

template std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
RadarEstimator::register_scan<pcl::PointXYZSDW>(
    const pcl::PointCloud<pcl::PointXYZSDW>::Ptr &) noexcept;

template std::tuple<std::vector<PlanarFeat>, std::vector<PointFeat>>
RadarEstimator::register_scan<pcl::PointHugin>(
    const pcl::PointCloud<pcl::PointHugin>::Ptr &) noexcept;

} // namespace rplrio
