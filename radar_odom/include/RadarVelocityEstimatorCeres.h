// PCL includes
#include "RadarVelocityEstimatorBase.h"
#include <ceres/ceres.h>

class DopplerCostFunction : public ceres::SizedCostFunction<1, 3>
{
private:
    double d_;
    Eigen::Vector3d p_;

public:
    DopplerCostFunction(Eigen::Vector3d pos, float doppler)
    {
        p_ = -pos / pos.norm();
        d_ = double(doppler);
    };

    ~DopplerCostFunction(){};
    bool Evaluate(double const *const *parameters,
                  double *residuals, double **jacobians) const
    {
        const double *v = parameters[0];

        residuals[0] = p_[0] * v[0] + p_[1] * v[1] + p_[2] * v[2] - d_;

        if (jacobians != nullptr && jacobians[0] != nullptr)
        {
            jacobians[0][0] = p_[0];
            jacobians[0][1] = p_[1];
            jacobians[0][2] = p_[2];
        }

        return true;
    };
};

class RadarVelocityEstimatorCeres : public RadarVelocityEstimatorBase<pcl::PointXYZSD>
{
public:
    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc, Eigen::Vector3d velocity);
};
