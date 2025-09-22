// PCL includes
#include "RadarVelocityEstimatorBase.h"
#include <ceres/ceres.h>
#include "RadarPointType.h"

class DopplerCostFunction : public ceres::SizedCostFunction<1, 3> {
private:
  double d_;
  Eigen::Vector3d p_;
  double s_ = 1;

public:
  DopplerCostFunction(Eigen::Vector3d pos, float doppler, double sigma) {
    p_ = -pos / pos.norm();
    d_ = double(doppler);
    s_ = sigma;
  };

  ~DopplerCostFunction() {};
  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const {
    const double *v = parameters[0];

    residuals[0] = (p_[0] * v[0] + p_[1] * v[1] + p_[2] * v[2] - d_) / s_;

    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = p_[0] / s_;
      jacobians[0][1] = p_[1] / s_;
      jacobians[0][2] = p_[2] / s_;
    }

    return true;
  };
};

class RadarVelocityEstimatorCeres
    : public RadarVelocityEstimatorBase<pcl::PointXYZSD> {
public:
  Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc);
  Eigen::Matrix3d
  get_covariance(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc,
                 Eigen::Vector3d velocity);
};
