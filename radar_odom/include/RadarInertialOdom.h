
// std includes
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

// PCL includes
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pclomp/ndt_omp.h>
#include <pclradar/RadarPointType.h>

// GTSAM includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include "DopplerMeasurementFactor.h"
#include "DopplerPoseMeasurementFactor.h"
#include "VelocityMeasurementFactor.h"
#include "VelocityPoseMeasurementFactor.h"
#include "AltitudeFactor.h"
#include "utils.h"

using namespace ros;
using namespace gtsam;
using namespace std;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

typedef boost::shared_ptr<PreintegratedCombinedMeasurements::Params> ImuParam_ptr;

ImuParam_ptr imuParams() // EPSON G364
{
    // use the sensor specs to build the noise model for the IMU factor.

    // The units for stddev are σ = m/s²/√Hz
    double accel_noise_sigma = 0.00006; // 0.06 mG/√Hz

    // The units for stddev are σ = rad/s/√Hz
    double gyro_noise_sigma = 0.00003684583; // 7.6 degree/h/√Hz --> 7.6 / 180 * pi / 3600

    // ? Velocity Random Walk 0.025 (m/s)/√h --> 0.025/√3600
    double accel_bias_rw_sigma = 0.00041666666;

    // 0.09 degree/√h --> 0.09 / 180 * pi / √3600
    double gyro_bias_rw_sigma = 0.00002617993;

    Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov = I_3x3 * 1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_init = I_6x6 * 1e-5; // error in the bias used for preintegration

    const double kGravity = -9.801;

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

class RadarIO
{
private:
    string imu_topic = "/epson_imu";
    string radar_topic = "/radar_points";

    Rot3 tf_imu2radar;
    Rot3 tf_base2map;
    double pre_imu_t = 0;
    int correction_count = 0;
    int imu_count = 0;
    bool correct_now = false;

    NodeHandle *nh;
    Subscriber sub_radar;
    Subscriber sub_imu;
    Publisher pub_odom;
    Publisher pub_v;
    tf::TransformBroadcaster br;

    // radar points
    pclomp::NormalDistributionsTransform<pcl::PointXYZSD, pcl::PointXYZSD> ndt_omp;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr pre_pc;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr radar_pc;
    pcl::PointCloud<pcl::PointXYZSD>::Ptr inlier_pc;

    // imu settings
    ImuParam_ptr imu_param = imuParams();
    std::shared_ptr<PreintegrationType> preintegrated;

    // states
    Values initial_values;
    NavState prev_state;
    NavState prop_state;
    imuBias::ConstantBias prev_bias;
    FixedLagSmoother::KeyTimestampMap key_time_stamps;

    // noise models
    noiseModel::Robust::shared_ptr radar_doppler_noise;
    noiseModel::Diagonal::shared_ptr radar_velocity_noise;
    noiseModel::Diagonal::shared_ptr radar_pose_noise;
    noiseModel::Diagonal::shared_ptr pose_noise_model;
    noiseModel::Isotropic::shared_ptr velocity_noise_model;
    noiseModel::Isotropic::shared_ptr bias_noise_model;
    noiseModel::Diagonal::shared_ptr altitude_noise_model;

    // optimizer & graph
    ISAM2 *isam2;
    IncrementalFixedLagSmoother *smootherISAM2;
    NonlinearFactorGraph *graph;

    void cb_radar(const sensor_msgs::PointCloud2 pc_msg);
    void cb_imu(const sensor_msgs::Imu imu);

public:
    RadarIO(NodeHandle *nh);
    ~RadarIO();
};
