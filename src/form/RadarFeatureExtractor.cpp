#define PCL_NO_PRECOMPILE
#include "form/RadarFeatureExtractor.h"
#include "RadarPointType.h"

#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <omp.h>

namespace rplrio {

template <typename PointT>
std::vector<bool> RadarFeatureExtractor::compute_valid_mask(
    const typename pcl::PointCloud<PointT>::Ptr &cloud) const {
  const size_t n = cloud->size();
  std::vector<bool> mask(n, true);

#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < n; ++i) {
    const auto &pt = (*cloud)[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      mask[i] = false;
      continue;
    }
    double range_sq = static_cast<double>(pt.x) * pt.x +
                      static_cast<double>(pt.y) * pt.y +
                      static_cast<double>(pt.z) * pt.z;
    if (range_sq < params.min_range * params.min_range ||
        range_sq > params.max_range * params.max_range) {
      mask[i] = false;
    }
  }
  return mask;
}

template <typename PointT>
std::vector<double> RadarFeatureExtractor::compute_curvature(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const pcl::KdTreeFLANN<PointT> &kdtree,
    const std::vector<bool> &valid_mask,
    std::vector<std::optional<Eigen::Vector3d>> &out_normals) const {

  const size_t n = cloud->size();
  std::vector<double> curvature(n, std::numeric_limits<double>::max());
  out_normals.assign(n, std::nullopt);

#pragma omp parallel
  {
    // Thread-local KNN buffers
    pcl::Indices nn_indices(params.k_neighbors);
    std::vector<float> nn_dists(params.k_neighbors);

#pragma omp for schedule(dynamic, 64)
    for (size_t i = 0; i < n; ++i) {
      if (!valid_mask[i])
        continue;

      int found = kdtree.nearestKSearch((*cloud)[i], params.k_neighbors,
                                        nn_indices, nn_dists);
      if (found < static_cast<int>(params.min_neighbors_for_normal))
        continue;

      // Centroid
      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (int j = 0; j < found; ++j) {
        const auto &nb = (*cloud)[nn_indices[j]];
        centroid += Eigen::Vector3d(nb.x, nb.y, nb.z);
      }
      centroid /= found;

      // Covariance
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for (int j = 0; j < found; ++j) {
        const auto &nb = (*cloud)[nn_indices[j]];
        Eigen::Vector3d diff = Eigen::Vector3d(nb.x, nb.y, nb.z) - centroid;
        cov += diff * diff.transpose();
      }
      cov /= found;

      // Eigendecomposition — eigenvalues in ascending order
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
          cov, Eigen::ComputeEigenvectors);

      // Smallest eigenvalue as planarity measure; corresponding eigenvector as normal
      curvature[i] = solver.eigenvalues()(0);
      out_normals[i] = solver.eigenvectors().col(0).normalized();
    }
  }
  return curvature;
}

template <typename PointT>
std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
RadarFeatureExtractor::extract(
    const typename pcl::PointCloud<PointT>::Ptr &cloud, size_t scan_idx) const {

  if (!cloud || cloud->empty()) {
    return {{}, {}};
  }

  // Build KD-tree (single-threaded build)
  pcl::KdTreeFLANN<PointT> kdtree;
  kdtree.setInputCloud(cloud);

  // Parallel: valid mask, curvature (smallest COV eigenvalue), and normals
  auto valid_mask = compute_valid_mask<PointT>(cloud);
  std::vector<std::optional<Eigen::Vector3d>> normals;
  auto curvature = compute_curvature<PointT>(cloud, kdtree, valid_mask, normals);

  // Sort indices by curvature (ascending)
  std::vector<size_t> sorted_indices;
  sorted_indices.reserve(cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    if (valid_mask[i] && curvature[i] < std::numeric_limits<double>::max()) {
      sorted_indices.push_back(i);
    }
  }
  std::sort(
      sorted_indices.begin(), sorted_indices.end(),
      [&curvature](size_t a, size_t b) { return curvature[a] < curvature[b]; });

  // --- Phase 1: Greedy select planar candidate indices (serial, cheap) ---
  std::vector<bool> used(cloud->size(), false);
  std::vector<size_t> planar_candidates;
  planar_candidates.reserve(params.max_planar_feats);

  pcl::Indices nn_indices(params.k_neighbors);
  std::vector<float> nn_dists(params.k_neighbors);

  for (size_t idx : sorted_indices) {
    if (planar_candidates.size() >= params.max_planar_feats)
      break;
    if (used[idx])
      continue;
    if (curvature[idx] >= params.planar_threshold)
      break;

    planar_candidates.push_back(idx);

    // Mark this point and its neighbors as used
    used[idx] = true;
    const auto &pt = (*cloud)[idx];
    int found =
        kdtree.nearestKSearch(pt, params.k_neighbors, nn_indices, nn_dists);
    for (int j = 0; j < found; ++j) {
      used[nn_indices[j]] = true;
    }
  }

  // --- Phase 2: Collect valid planar features (normals already computed) ---
  const size_t num_candidates = planar_candidates.size();
  std::vector<form::PlanarFeat> planar_feats;
  planar_feats.reserve(num_candidates);
  for (size_t i = 0; i < num_candidates; ++i) {
    const size_t idx = planar_candidates[i];
    if (!normals[idx].has_value())
      continue;
    const auto &pt = (*cloud)[idx];
    const auto &n = normals[idx].value();
    planar_feats.emplace_back(
        static_cast<double>(pt.x), static_cast<double>(pt.y),
        static_cast<double>(pt.z), n.x(), n.y(), n.z(), scan_idx);
  }

  // --- Phase 4: Extract point features (remaining unused) ---
  std::vector<form::PointFeat> point_feats;
  point_feats.reserve(params.max_point_feats);
  for (size_t idx : sorted_indices) {
    if (point_feats.size() >= params.max_point_feats)
      break;
    if (used[idx])
      continue;

    const auto &pt = (*cloud)[idx];
    point_feats.emplace_back(static_cast<double>(pt.x),
                             static_cast<double>(pt.y),
                             static_cast<double>(pt.z), scan_idx);
    used[idx] = true;
  }

  return {std::move(planar_feats), std::move(point_feats)};
}

// Explicit instantiations
template std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
RadarFeatureExtractor::extract<pcl::PointXYZD>(
    const pcl::PointCloud<pcl::PointXYZD>::Ptr &, size_t) const;

template std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
RadarFeatureExtractor::extract<pcl::PointXYZSD>(
    const pcl::PointCloud<pcl::PointXYZSD>::Ptr &, size_t) const;

template std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
RadarFeatureExtractor::extract<pcl::PointXYZSDW>(
    const pcl::PointCloud<pcl::PointXYZSDW>::Ptr &, size_t) const;

template std::tuple<std::vector<form::PlanarFeat>, std::vector<form::PointFeat>>
RadarFeatureExtractor::extract<pcl::PointHugin>(
    const pcl::PointCloud<pcl::PointHugin>::Ptr &, size_t) const;

} // namespace rplrio
