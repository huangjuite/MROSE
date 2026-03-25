#pragma once

#include "form/RadarFeatureExtractor.hpp"

#include "form/feature/features.hpp"
#include "form/mapping/keyscanner.hpp"
#include "form/mapping/map.hpp"
#include "form/optimization/constraints.hpp"
#include "form/optimization/matcher.hpp"

#include <gtsam/geometry/Pose3.h>
#include <pcl/point_cloud.h>

#include <tuple>
#include <vector>

namespace rplrio {

struct RadarEstimator {
  struct Params {
    RadarFeatureExtractor::Params extraction;
    form::MatcherParams matcher;
    form::ConstraintManager::Params constraints;
    form::KeyScanner::Params scans;
    form::KeypointMapParams map;
  };

  Params m_params;

  RadarFeatureExtractor m_extractor;

  form::ConstraintManager m_constraints;
  std::tuple<form::Matcher<form::PlanarFeat>, form::Matcher<form::PointFeat>> m_matcher;

  form::KeyScanner m_keyscanner;
  std::tuple<form::KeypointMap<form::PlanarFeat>, form::KeypointMap<form::PointFeat>> m_keypoint_map;

  RadarEstimator() : RadarEstimator(Params()) {}
  RadarEstimator(const Params &params) noexcept;

  gtsam::Pose3 current_lidar_estimate() { return m_constraints.get_current_pose(); }

  template <typename PointT>
  std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
  register_scan(const typename pcl::PointCloud<PointT>::Ptr &cloud) noexcept;
};

} // namespace rplrio
