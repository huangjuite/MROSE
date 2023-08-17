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
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

// custom factor
#include "DopplerMeasurementFactor.h"

using namespace gtsam;

class RadarVelocityEstimator
{
private:
    SharedNoiseModel doppler_noise_model;
    SharedNoiseModel prior_noise_model;
    Vector3 initial_v;
    Matrix covariance_matrix;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr inlier_pc;
    float inlier_thresh;

    float doppler_projection(pcl::PointXYZSD p, Vector3 v);

public:
    RadarVelocityEstimator(SharedNoiseModel doppler_noiseModel);
    ~RadarVelocityEstimator();

    gtsam::Vector3 optimize(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc);
    Matrix get_covariance(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc, Vector3 velocity);
    pcl::PointCloud<pcl::PointXYZSD>::Ptr get_inlier(pcl::PointCloud<pcl::PointXYZSD>::ConstPtr radar_pc, Vector3 velocity);
    void set_prior_noise_model(SharedNoiseModel noise_model);
    void set_initial_velocity(Vector3 init_v);
    void set_inlier_thresh(float thresh);
};
