
// std includes
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "DopplerMeasurementFactor.h"
#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "AltitudeFactor.h"
#include "RotationFactor.h"
#include "RadarVelocityEstimator.h"
#include "RadarVelocityEstimatorHCS.h"
#include "RadarVelocityEstimatorCelestial.h"
#include "LandmarkFactor.h"
#include "LandmarkTracker.h"
#include "utils.h"

using namespace ros;
using namespace gtsam;
using namespace std;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::L; // Landmark (x,y,z)

typedef boost::shared_ptr<PreintegratedCombinedMeasurements::Params> ImuParam_ptr;

ImuParam_ptr imuParams() // EPSON G364
{
    // use the sensor specs to build the noise model for the IMU factor.

    // The units for stddev are σ = m/s²/√Hz
    // 0.06 mG/√Hz
    // double accel_noise_sigma = 0.00006;
    double accel_noise_sigma = 0.0015903917463388937;

    // The units for stddev are σ = rad/s/√Hz
    // 7.6 degree/h/√Hz --> 7.6 / 180 * pi / 3600
    // double gyro_noise_sigma = 0.00003684583;
    double gyro_noise_sigma = 3.581400308085914e-05;

    // ? Velocity Random Walk 0.025 (m/s)/√h --> 0.025/√3600
    // double accel_bias_rw_sigma = 0.00041666666;
    double accel_bias_rw_sigma = 7.316842828686425e-05;

    // 0.09 degree/√h --> 0.09 / 180 * pi / √3600
    // double gyro_bias_rw_sigma = 0.00002617993;
    double gyro_bias_rw_sigma = 1.1036348379504944e-06;

    Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov = I_3x3 * 1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_init = I_6x6 * 1e-5; // error in the bias used for preintegration

    const double kGravity = -9.81;

    auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(kGravity);
    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov;    // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov;     // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_init;

    return p;
}

class DopplerLandmarkIO
{
private:
    string imu_topic = "/epson_imu";
    string radar_topic = "/radar_points";
    float doppler_resolution = 0.05512325; // meter/s
    double lag = 2.0;                      // seconds

    Rot3 tf_imu2radar;
    Rot3 tf_base2map;
    double pre_imu_t = 0;
    int step = 0;
    int imu_count = 0;
    bool correct_now = false;

    vector<vector<int>> colors = {
        {120, 28, 129}, {64, 67, 153}, {72, 139, 194}, {107, 178, 140}, {159, 190, 87}, {210, 179, 63}, {231, 126, 49}, {217, 33, 32}};

    NodeHandle *nh;
    Subscriber sub_radar;
    Subscriber sub_imu;
    Publisher pub_odom;
    Publisher pub_v;
    Publisher pub_twist;
    Publisher pub_bias;
    Publisher pub_radar;
    Publisher pub_error;
    Publisher pub_landmark;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    // radar
    RadarVelocityEstimator *velocity_estimator;
    // RadarVelocityEstimatorCelestial *velocity_estimator;
    // RadarVelocityEstimatorHCS *velocity_estimator;
    pcl::PointCloud<pcl::PointXYZIS>::Ptr radar_pc;
    LandmarkTracker tracker;

    // imu settings
    ImuParam_ptr imu_param = imuParams();
    std::shared_ptr<PreintegrationType> preintegrated;

    // states
    Values initial_values;
    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;

    // noise models
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Isotropic::shared_ptr velocity_noise_model;
    noiseModel::Isotropic::shared_ptr bias_noise_model;
    // noiseModel::Diagonal::shared_ptr landmark_noise_model;

    // optimizer & graph
    FixedLagSmoother::KeyTimestampMap key_timestamps;
    IncrementalFixedLagSmoother *smootherISAM2;
    NonlinearFactorGraph *graph;

    void cb_radar(const sensor_msgs::PointCloud2 pc_msg);
    void cb_imu(const sensor_msgs::Imu imu);
    std_msgs::ColorRGBA id_to_color(uint id);

public:
    DopplerLandmarkIO(NodeHandle *nh);
    ~DopplerLandmarkIO();

    // tmp
    ofstream err_file;
};
