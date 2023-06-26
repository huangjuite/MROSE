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

        Eigen::Matrix4d T_i_b;
        Eigen::Matrix3d R_i_b;
        double acc_noise;
        double acc_rw;
        double gyro_noise;
        double gyro_rw;

        Eigen::Matrix4d T_r_i;
        Eigen::Matrix4d T_r_b;
        Eigen::Matrix3d R_r_b;
        double doppler_std;
        double azimuth_std;
        double elevation_std;

        double init_window;
        double init_thres;
        double lag;

        double gravity;
    };

    Params get_params(ros::NodeHandle &nh)
    {
        Params param;
        std::string radar_, imu_;
        nh.getParam("/odom/imu", imu_);
        nh.getParam("/odom/radar", radar_);
        nh.getParam(radar_ + std::string("/rostopic"), param.radar_topic);
        nh.getParam(imu_ + std::string("/rostopic"), param.imu_topic);

        nh.getParam(imu_ + std::string("/accelerometer_noise_density"), param.acc_noise);
        nh.getParam(imu_ + std::string("/accelerometer_random_walk"), param.acc_rw);
        nh.getParam(imu_ + std::string("/gyroscope_noise_density"), param.gyro_noise);
        nh.getParam(imu_ + std::string("/gyroscope_random_walk"), param.gyro_rw);
        nh.getParam(imu_ + std::string("/gravity"), param.gravity);

        XmlRpc::XmlRpcValue tf;
        nh.getParam(imu_ + std::string("/T_i_b"), tf);
        for (uint i = 0; i < tf.size(); i++)
        {
            for (uint j = 0; j < tf[i].size(); j++)
            {
                param.T_i_b(i, j) = tf[i][j];
            }
        }
        param.R_i_b = param.T_i_b.block(0, 0, 3, 3);

        nh.getParam(radar_ + std::string("/doppler_sigma"), param.doppler_std);
        nh.getParam(radar_ + std::string("/azimuth_sigma"), param.azimuth_std);
        nh.getParam(radar_ + std::string("/elevation_sigma"), param.elevation_std);
        nh.getParam(radar_ + std::string("/T_r_i"), tf);
        for (uint i = 0; i < tf.size(); i++)
        {
            for (uint j = 0; j < tf[i].size(); j++)
            {
                param.T_r_i(i, j) = tf[i][j];
            }
        }
        param.T_r_b = param.T_i_b * param.T_r_i;
        param.R_r_b = param.T_r_b.block(0, 0, 3, 3);

        nh.getParam("/odom/static_init_window", param.init_window);
        nh.getParam("/odom/static_init_threshold", param.init_thres);
        nh.getParam("/odom/optimization_window", param.lag);

        return param;
    }

    ImuParam_ptr imuParams(Params param) // EPSON G364
    {
        gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(param.acc_noise, 2);
        gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(param.gyro_noise, 2);
        gtsam::Matrix33 integration_error_cov = gtsam::I_3x3 * 1e-8; // error committed in integrating position from velocities
        gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(param.acc_rw, 2);
        gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(param.gyro_rw, 2);
        gtsam::Matrix66 bias_acc_omega_init = gtsam::I_6x6 * 1e-5; // error in the bias used for preintegration

        auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(param.gravity);
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
