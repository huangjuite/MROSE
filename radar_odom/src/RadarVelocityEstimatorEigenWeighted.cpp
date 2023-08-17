#include "RadarVelocityEstimatorEigenWeighted.h"

Eigen::Vector3d RadarVelocityEstimatorEigenWeighted::optimize(
    pcl::PointCloud<pcl::PointXYZSDW>::ConstPtr radar_pc)
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
        d(i, 0) = p.weight * p.doppler;
    }

    Eigen::Vector3d v_est = P.colPivHouseholderQr().solve(d);

    return v_est;
}

Eigen::Matrix3d RadarVelocityEstimatorEigenWeighted::get_covariance(
    pcl::PointCloud<pcl::PointXYZSDW>::ConstPtr radar_pc, Eigen::Vector3d velocity)
{

    size_t n = radar_pc->size();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n, 3);
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(n, 1);
    Eigen::MatrixXd residual = Eigen::MatrixXd::Zero(n, 1);

    double d_std = 0.15287400537531382;
    double d_var = d_std * d_std;

    double err = 0;

    for (size_t i = 0; i < n; i++)
    {
        auto p = radar_pc->points[i];
        Eigen::Vector3d pn(p.x, p.y, p.z);
        pn = p.weight * -pn / pn.norm();
        P(i, 0) = pn[0];
        P(i, 1) = pn[1];
        P(i, 2) = pn[2];

        // d(i, 0) = p.doppler;
        // residual(i, 0) = pn.dot(velocity) - d(i, 0);
        // err += residual(i, 0) * residual(i, 0);

    }

    // Eigen::MatrixXd R = (P.transpose() * P).inverse() * P.transpose();
    // Eigen::MatrixXd var = residual * residual.transpose() / n;
    // Eigen::Matrix3d covariance_matrix = R * var * R.transpose();

    Eigen::Matrix3d covariance_matrix = d_var * (P.transpose() * P).inverse();

    return covariance_matrix;
}
