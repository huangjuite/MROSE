// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

#include "RadarVelocityEstimatorBase.h"

// custom factor
#include "DopplerCelestial2Factor.h"

using gtsam::symbol_shorthand::C; // celestial coordinate

class RadarVelocityEstimatorGtsamHCS : public RadarVelocityEstimatorBase<pcl::PointXYZIS>
{
private:
    gtsam::SharedNoiseModel doppler_noise_model;
    gtsam::SharedNoiseModel angle_noise_model;

    gtsam::Vector2 toCelestial(pcl::PointXYZIS &p);

public:
    void set_uncertainty(
        double doppler_std, double azimuth_std, double elevation_std);

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(
        pcl::PointCloud<pcl::PointXYZIS>::ConstPtr radar_pc, Eigen::Vector3d velocity);
};
