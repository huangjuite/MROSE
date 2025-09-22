#ifndef RADAR_VEST_BASE_H
#define RADAR_VEST_BASE_H

// PCL includes
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

template <class PointType> class RadarVelocityEstimatorBase {
protected:
  typedef typename pcl::PointCloud<PointType>::ConstPtr PclConstPtr;
  typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
  typedef typename pcl::PointCloud<PointType> Pcl;

  Eigen::Vector3d initial_v;
  Eigen::Matrix3d covariance_matrix;
  float inlier_thresh = 0.1;

  float doppler_projection(PointType p, Eigen::Vector3d v) {
    Eigen::Vector3d unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
  }

public:
  RadarVelocityEstimatorBase() {};
  ~RadarVelocityEstimatorBase() {};

  void set_initial_velocity(Eigen::Vector3d init_v) {
    this->initial_v = init_v;
  };

  void set_inlier_thresh(float thresh) { this->inlier_thresh = thresh; };

  PclPtr get_inlier(PclConstPtr radar_pc, Eigen::Vector3d velocity) {
    PclPtr inlier_pc(new Pcl);
    for (auto p : radar_pc->points) {
      float err = abs(doppler_projection(p, velocity) - p.doppler);
      if (err < inlier_thresh) {
        inlier_pc->push_back(p);
      }
    }

    return inlier_pc;
  };

  virtual Eigen::Vector3d optimize(PclConstPtr radar_pc) = 0;

  virtual Eigen::Matrix3d get_covariance(PclConstPtr radar_pc,
                                         Eigen::Vector3d velocity) = 0;
};

#endif