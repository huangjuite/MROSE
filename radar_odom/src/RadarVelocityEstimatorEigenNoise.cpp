#include "RadarVelocityEstimatorEigenNoise.h"

RadarVelocityEstimatorEigenNoise::RadarVelocityEstimatorEigenNoise()
{
    this->inlier_thresh = 0.1;
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZISW>);
}

void RadarVelocityEstimatorEigenNoise::set_inlier_thresh(float thresh)
{
    this->inlier_thresh = thresh;
}

void RadarVelocityEstimatorEigenNoise::set_initial_velocity(Eigen::Vector3d init_v)
{
    this->initial_v = init_v;
}

float RadarVelocityEstimatorEigenNoise::doppler_projection(pcl::PointXYZISW p, Eigen::Vector3d v)
{
    Eigen::Vector3d unit_r(p.x, p.y, p.z);
    unit_r = -unit_r / unit_r.norm();
    return unit_r.dot(v);
}

Eigen::Vector3d RadarVelocityEstimatorEigenNoise::optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc)
{

    size_t n = radar_pc->size();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(n, 1);

    for (size_t i = 0; i < n; i++)
    {
        auto p = radar_pc->points[i];
        Eigen::Vector3d pn(p.x, p.y, p.z);
        pn = p.weight * -pn / pn.norm();
        P(i, 0) = pn[0];
        P(i, 1) = pn[1];
        P(i, 2) = pn[2];
        d(i, 0) = p.weight * p.speed;
    }

    Eigen::Vector3d v_est = P.colPivHouseholderQr().solve(d);

    return v_est;
}

pcl::PointCloud<pcl::PointXYZISW>::Ptr RadarVelocityEstimatorEigenNoise::get_inlier(
    pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{
    // collect inliers
    inlier_pc.reset(new pcl::PointCloud<pcl::PointXYZISW>);
    for (pcl::PointXYZISW p : radar_pc->points)
    {
        float err = abs(doppler_projection(p, velocity) - p.speed);
        if (err < inlier_thresh)
        {
            inlier_pc->push_back(p);
        }
    }

    return inlier_pc;
}

Eigen::Matrix3d RadarVelocityEstimatorEigenNoise::get_covariance(
    pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{

    size_t n = radar_pc->size();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(n, 1);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(n);
    double err = 0;
    for (size_t i = 0; i < n; i++)
    {
        auto p = radar_pc->points[i];
        Eigen::Vector3d pn(p.x, p.y, p.z);
        pn =  -pn / pn.norm();
        P(i, 0) = pn[0];
        P(i, 1) = pn[1];
        P(i, 2) = pn[2];
        d(i, 0) = p.weight * p.speed;

        residual[i] = (pn.dot(velocity) - d(i, 0));
        err += residual[i] * residual[i];
    }

    double variance = err / n;
    covariance_matrix = (P.transpose() * P).inverse();

    return covariance_matrix;
}

RadarVelocityEstimatorEigenNoise::~RadarVelocityEstimatorEigenNoise()
{
}
