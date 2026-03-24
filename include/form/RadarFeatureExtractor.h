#pragma once

#include <vector>
#include <tuple>
#include <optional>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "form/feature/features.hpp"

namespace rplrio {

class RadarFeatureExtractor {
public:
  struct Params {
    // KNN parameters
    int k_neighbors = 50;

    // Curvature thresholds
    double planar_threshold = 0.1;

    // Feature limits per scan
    size_t max_planar_feats = 300;
    size_t max_point_feats = 0;

    // Range filter (in meters)
    double min_range = 1.0;
    double max_range = 300.0;

    // Normal estimation
    size_t min_neighbors_for_normal = 5;
  };

  Params params;

  explicit RadarFeatureExtractor(const Params &params) : params(params) {}

  template <typename PointT>
  [[nodiscard]] std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
  extract(const typename pcl::PointCloud<PointT>::Ptr &cloud, size_t scan_idx) const;

private:
  // Computes the smallest eigenvalue of the neighbourhood covariance matrix as
  // a planarity measure, and stores the corresponding eigenvector (surface
  // normal) in out_normals.  Points whose neighbourhood is too small are left
  // with curvature == max and out_normals == nullopt.
  template <typename PointT>
  [[nodiscard]] std::vector<double>
  compute_curvature(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                    const pcl::KdTreeFLANN<PointT> &kdtree,
                    const std::vector<bool> &valid_mask,
                    std::vector<std::optional<Eigen::Vector3d>> &out_normals) const;

  template <typename PointT>
  [[nodiscard]] std::vector<bool>
  compute_valid_mask(const typename pcl::PointCloud<PointT>::Ptr &cloud) const;
};

} // namespace rplrio