#include "RadarVelocityEstimatorBase.h"
#include <ceres/ceres.h>


class DopplerCostFunctionWeighted : public ceres::SizedCostFunction<1, 3>
{
private:
    double d_;
    double w_;
    Eigen::Vector3d p_;

public:
    DopplerCostFunctionWeighted(Eigen::Vector3d pos, float doppler, double weight)
    {
        p_ = -pos / pos.norm();
        d_ = double(doppler);
        w_ = weight;
    };

    ~DopplerCostFunctionWeighted(){};
    bool Evaluate(double const *const *parameters,
                  double *residuals, double **jacobians) const
    {
        const double *v = parameters[0];

        residuals[0] = w_ * (p_[0] * v[0] + p_[1] * v[1] + p_[2] * v[2] - d_);

        if (jacobians != nullptr && jacobians[0] != nullptr)
        {
            jacobians[0][0] = sqrt(w_) * p_[0];
            jacobians[0][1] = sqrt(w_) * p_[1];
            jacobians[0][2] = sqrt(w_) * p_[2];
        }

        return true;
    };
};

class RadarVelocityEstimatorCeresWeighted : public RadarVelocityEstimatorBase<pcl::PointXYZISW>
{
public:

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
};
