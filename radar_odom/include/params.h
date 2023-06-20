#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>

typedef boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> ImuParam_ptr;

namespace rplrio
{
    struct Params
    {
        std::string radar_topic;
        std::string imu_topic;

        double acc_noise;
        double acc_rw;
        double gyro_noise;
        double gyro_rw;

        Eigen::Matrix3d R_i_r;
        double doppler_std;
        double azimuth_std;
        double elevation_std;
        
        double init_window;
        double init_thres;
        double lag;
    };

    Params get_params(ros::NodeHandle &nh)
    {
        Params param;
        nh.getParam("/radar0/rostopic", param.radar_topic);
        nh.getParam("/imu0/rostopic", param.imu_topic);

        nh.getParam("/imu0/accelerometer_noise_density", param.acc_noise);
        nh.getParam("/imu0/accelerometer_random_walk", param.acc_rw);
        nh.getParam("/imu0/gyroscope_noise_density", param.gyro_noise);
        nh.getParam("/imu0/gyroscope_random_walk", param.gyro_rw);

        nh.getParam("/radar0/doppler_sigma", param.doppler_std);
        nh.getParam("/radar0/azimuth_sigma", param.azimuth_std);
        nh.getParam("/radar0/elevation_sigma", param.elevation_std);
        nh.getParam("/odom/static_init_window", param.init_window);
        nh.getParam("/odom/static_init_threshold", param.init_thres);
        nh.getParam("/odom/optimization_window", param.lag);

        XmlRpc::XmlRpcValue tf;
        nh.getParam(std::string("radar0/T_i_r"), tf);
        Eigen::Matrix4d T;
        for (uint i = 0; i < tf.size(); i++)
        {
            for (uint j = 0; j < tf[i].size(); j++)
            {
                T(i, j) = tf[i][j];
            }
        }
        param.R_i_r = T.block(0, 0, 3, 3);

        return param;
    }

    ImuParam_ptr imuParams(Params param) // EPSON G364
    {
        // use the sensor specs to build the noise model for the IMU factor.

        // The units for stddev are σ = m/s²/√Hz
        // 0.06 mG/√Hz
        // double accel_noise_sigma = 0.00006;
        double accel_noise_sigma = param.acc_noise;

        // The units for stddev are σ = rad/s/√Hz
        // 7.6 degree/h/√Hz --> 7.6 / 180 * pi / 3600
        // double gyro_noise_sigma = 0.00003684583;
        double gyro_noise_sigma = param.gyro_noise;

        // ? Velocity Random Walk 0.025 (m/s)/√h --> 0.025/√3600
        // double accel_bias_rw_sigma = 0.00041666666;
        double accel_bias_rw_sigma = param.acc_rw;

        // 0.09 degree/√h --> 0.09 / 180 * pi / √3600
        // double gyro_bias_rw_sigma = 0.00002617993;
        double gyro_bias_rw_sigma = param.gyro_rw;

        gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(accel_noise_sigma, 2);
        gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(gyro_noise_sigma, 2);
        gtsam::Matrix33 integration_error_cov = gtsam::I_3x3 * 1e-8; // error committed in integrating position from velocities
        gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(accel_bias_rw_sigma, 2);
        gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(gyro_bias_rw_sigma, 2);
        gtsam::Matrix66 bias_acc_omega_init = gtsam::I_6x6 * 1e-5; // error in the bias used for preintegration

        const double kGravity = 9.81;

        auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(kGravity);
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

};

#endif
