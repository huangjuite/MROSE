// PCL includes
#include <stdlib.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

#include <Eigen/Dense>
#include <ceres/ceres.h>


class DopplerCostFunctionNoise : public ceres::SizedCostFunction<1, 3>
{
private:
    double d_;
    double w_;
    Eigen::Vector3d p_;

public:
    DopplerCostFunctionNoise(Eigen::Vector3d pos, float doppler, double weight)
    {
        p_ = -pos / pos.norm();
        d_ = double(doppler);
        w_ = weight;
    };

    ~DopplerCostFunctionNoise(){};
    bool Evaluate(double const *const *parameters,
                  double *residuals, double **jacobians) const
    {
        const double *v = parameters[0];

        residuals[0] = w_ * (p_[0] * v[0] + p_[1] * v[1] + p_[2] * v[2] - d_);

        if (jacobians != nullptr && jacobians[0] != nullptr)
        {
            jacobians[0][0] = w_ * p_[0];
            jacobians[0][1] = w_ * p_[1];
            jacobians[0][2] = w_ * p_[2];
        }

        return true;
    };
};

class RadarVelocityEstimatorCeresNoise
{
private:
    Eigen::Vector3d initial_v;
    Eigen::Matrix3d covariance_matrix;
    pcl::PointCloud<pcl::PointXYZISW>::Ptr inlier_pc;
    float inlier_thresh;

    float doppler_projection(pcl::PointXYZISW p, Eigen::Vector3d v);

public:
    RadarVelocityEstimatorCeresNoise();
    ~RadarVelocityEstimatorCeresNoise();

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    pcl::PointCloud<pcl::PointXYZISW>::Ptr get_inlier(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    void set_initial_velocity(Eigen::Vector3d init_v);
    void set_inlier_thresh(float thresh);
};
