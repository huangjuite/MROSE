// PCL includes
#include <stdlib.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

#include <Eigen/Dense>
#include <ceres/ceres.h>

class DopplerCostFunctor
{
private:
    double d_;
    Eigen::Vector3d p_;

public:
    DopplerCostFunctor(Eigen::Vector3d pos, float doppler)
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

class DopplerCostFunction : public ceres::SizedCostFunction<1, 3>
{
private:
    double d_;
    double s_;
    Eigen::Vector3d p_;

public:
    DopplerCostFunction(Eigen::Vector3d pos, float doppler, double sigma)
    {
        p_ = -pos / pos.norm();
        d_ = double(doppler);
        s_ = sigma;
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

class RadarVelocityEstimatorCeres
{
private:
    Eigen::Vector3d initial_v;
    Eigen::Matrix3d covariance_matrix;
    pcl::PointCloud<pcl::PointXYZIS>::Ptr inlier_pc;
    float inlier_thresh;

    float doppler_projection(pcl::PointXYZIS p, Eigen::Vector3d v);

public:
    RadarVelocityEstimatorCeres();
    ~RadarVelocityEstimatorCeres();

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    pcl::PointCloud<pcl::PointXYZIS>::Ptr get_inlier(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    void set_initial_velocity(Eigen::Vector3d init_v);
    void set_inlier_thresh(float thresh);
};
