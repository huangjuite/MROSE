#pragma once

// PCL includes
#include <fstream>
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

template <typename PointType> class RadarVelocityEstimatorBase {
protected:
  typedef typename pcl::PointCloud<PointType>::ConstPtr PclConstPtr;
  typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
  typedef typename pcl::PointCloud<PointType> Pcl;

  Eigen::Vector3d initial_v;
  Eigen::Matrix3d covariance_matrix;
  float inlier_thresh_ = 0.1f;
  float doppler_sigma_ = 0.1f;
  float min_pt_dist_ = 0.1f;
  std::ofstream *timing_csv_ = nullptr;

  float doppler_projection(PointType p, Eigen::Vector3d v) {
    Eigen::Vector3d unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
  }

public:
  RadarVelocityEstimatorBase(){};
  ~RadarVelocityEstimatorBase(){};

  void set_initial_velocity(Eigen::Vector3d init_v) {
    this->initial_v = init_v;
  };

  void set_inlier_thresh(float thresh) { this->inlier_thresh_ = thresh; };

  void set_doppler_sigma(float sigma) { this->doppler_sigma_ = sigma; };

  void set_timing_csv(std::ofstream *csv) { this->timing_csv_ = csv; };

  PclPtr get_inlier(PclConstPtr radar_pc, Eigen::Vector3d velocity) {
    PclPtr inlier_pc(new Pcl);
    for (auto p : radar_pc->points) {
      float err = abs(doppler_projection(p, velocity) - p.doppler);
      if (err < inlier_thresh_) {
        inlier_pc->push_back(p);
      }
    }

    return inlier_pc;
  };

  virtual Eigen::Vector3d optimize(PclConstPtr radar_pc) = 0;

  virtual Eigen::Matrix3d get_covariance(PclConstPtr radar_pc,
                                         Eigen::Vector3d velocity) = 0;
};
