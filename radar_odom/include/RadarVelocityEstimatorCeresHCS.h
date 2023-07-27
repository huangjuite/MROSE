
#include "RadarVelocityEstimatorBase.h"
#include <ceres/ceres.h>
#include <ceres/normal_prior.h>

#include "params.h"

class DopplerAngleCostFunction : public ceres::SizedCostFunction<1, 3, 2>
{
private:
    double d_;
    double s_;

public:
    DopplerAngleCostFunction(float doppler, double sigma)
    {
        d_ = double(doppler);
        s_ = sigma;
    };

    ~DopplerAngleCostFunction(){};
    bool Evaluate(double const *const *parameters,
                  double *residuals, double **jacobians) const
    {
        const double *v = parameters[0];
        const double theta = parameters[1][0];
        const double phi = parameters[1][1];

        const double x = -ceres::cos(phi) * ceres::cos(theta);
        const double y = -ceres::cos(phi) * ceres::sin(theta);
        const double z = -ceres::sin(phi);
        residuals[0] = (x * v[0] + y * v[1] + z * v[2] - d_) / s_;

        if (jacobians != nullptr && jacobians[0] != nullptr)
        {
            jacobians[0][0] = x;
            jacobians[0][1] = y;
            jacobians[0][2] = z;
        }

        if (jacobians != nullptr && jacobians[1] != nullptr)
        {
            jacobians[1][0] = -ceres::cos(phi) * ceres::sin(theta) * v[0] +
                              ceres::cos(phi) * ceres::cos(theta) * v[1];
            jacobians[1][1] = -ceres::sin(phi) * (ceres::cos(theta) * v[0] + ceres::sin(theta) * v[1]) +
                              ceres::cos(phi) * v[2];
        }

        return true;
    };
};

class DopplerResidual
{
private:
    double d_;
    Eigen::Vector3d p_;

public:
    DopplerResidual(Eigen::Vector3d pos, float doppler)
    {
        p_ = -pos / pos.norm();
        d_ = double(doppler);
    };
    template <typename T>
    bool operator()(const T *const v, T *residual) const
    {
        residual[0] = p_(0) * v[0] + p_(1) * v[1] + p_(2) * v[2] - d_;
        return true;
    }
};

class RadarVelocityEstimatorCeresHCS : public RadarVelocityEstimatorBase<pcl::PointXYZIS>
{
private:
    rplrio::Params param;

public:
    RadarVelocityEstimatorCeresHCS(rplrio::Params &param);

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    static Eigen::Vector2d toHCS(pcl::PointXYZIS &p);
};
