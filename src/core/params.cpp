#include "core/params.hpp"
#include <yaml-cpp/yaml.h>

namespace rplrio {

ImuParam_ptr imuParams(Params param) // EPSON G364
{
  gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(param.acc_noise, 2);
  gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(param.gyro_noise, 2);
  gtsam::Matrix33 integration_error_cov =
      gtsam::I_3x3 *
      1e-8; // error committed in integrating position from velocities
  gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(param.acc_rw, 2);
  gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(param.gyro_rw, 2);
  gtsam::Matrix66 bias_acc_omega_init =
      gtsam::I_6x6 * 1e-5; // error in the bias used for preintegration

  auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(
      param.gravity);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov; // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov; // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;     // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_init;

  return p;
}
Params get_params(const std::string &cfg_path) {
  Params param;
  YAML::Node config = YAML::LoadFile(cfg_path);

  // imu
  std::string imu_ = config["odom"]["imu"].as<std::string>();
  param.imu_topic = config[imu_]["rostopic"].as<std::string>();
  param.acc_noise = config[imu_]["accelerometer_noise_density"].as<double>();
  param.acc_rw = config[imu_]["accelerometer_random_walk"].as<double>();
  param.gyro_noise = config[imu_]["gyroscope_noise_density"].as<double>();
  param.gyro_rw = config[imu_]["gyroscope_random_walk"].as<double>();
  param.gravity = config[imu_]["gravity"].as<double>();

  auto T_i_b_node = config[imu_]["T_i_b"];
  for (size_t i = 0; i < 4; ++i)
    for (size_t j = 0; j < 4; ++j)
      param.T_i_b(i, j) = T_i_b_node[i][j].as<double>();
  param.R_i_b = param.T_i_b.block(0, 0, 3, 3);

  // radar
  param.radar_list = config["odom"]["radar"].as<std::vector<std::string>>();
  for (const std::string &r : param.radar_list) {
    param.radars[r].topic = config[r]["rostopic"].as<std::string>();
    param.radars[r].doppler_std = config[r]["doppler_sigma"].as<double>();
    if (config[r]["type"])
      param.radars[r].type = config[r]["type"].as<std::string>();
    auto T_r_i_node = config[r]["T_r_i"];
    for (size_t i = 0; i < 4; ++i)
      for (size_t j = 0; j < 4; ++j)
        param.radars[r].T_r_i(i, j) = T_r_i_node[i][j].as<double>();
    param.radars[r].R_r_i = param.radars[r].T_r_i.block(0, 0, 3, 3);
  }

  // odom
  param.lag = config["odom"]["optimization_window"].as<double>();
  param.bv_use_huber = config["odom"]["bv_use_huber"].as<bool>();
  param.bv_huber_k = config["odom"]["bv_huber_k"].as<double>();

  if (config["odom"]["marginalization_sigma"]) {
    auto marg_sigma =
        config["odom"]["marginalization_sigma"].as<std::vector<double>>();
    param.marginalization_sigma = Eigen::VectorXd(marg_sigma.size());
    for (size_t i = 0; i < marg_sigma.size(); ++i)
      param.marginalization_sigma(i) = marg_sigma[i];
  } else {
    param.marginalization_sigma = Eigen::VectorXd(6);
    param.marginalization_sigma << 10, 10, 10, 10, 10, 10;
  }

  param.use_static_init = config["odom"]["use_static_init"].as<bool>();
  param.init_bias_acc = config["odom"]["init_bias_acc"]
      ? Eigen::Vector3d(config["odom"]["init_bias_acc"][0].as<double>(),
                        config["odom"]["init_bias_acc"][1].as<double>(),
                        config["odom"]["init_bias_acc"][2].as<double>())
      : Eigen::Vector3d::Zero();
  param.init_bias_gyro = config["odom"]["init_bias_gyro"]
      ? Eigen::Vector3d(config["odom"]["init_bias_gyro"][0].as<double>(),
                        config["odom"]["init_bias_gyro"][1].as<double>(),
                        config["odom"]["init_bias_gyro"][2].as<double>())
      : Eigen::Vector3d::Zero();
  param.doppler_inlier_thresh =
      config["odom"]["doppler_inlier_thresh"].as<double>();
  param.init_window = config["odom"]["static_init_window"].as<double>();
  param.init_thres = config["odom"]["static_init_threshold"].as<double>();

  if (config["log_level"]) {
    std::string lvl = config["log_level"].as<std::string>();
    if (lvl == "trace")         param.log_level = LogLevel::TRACE;
    else if (lvl == "debug")    param.log_level = LogLevel::DEBUG;
    else if (lvl == "info")     param.log_level = LogLevel::INFO;
    else if (lvl == "warn")     param.log_level = LogLevel::WARN;
    else if (lvl == "error")    param.log_level = LogLevel::ERROR;
    else if (lvl == "fatal" || lvl == "critical") param.log_level = LogLevel::CRITICAL;
  }

  return param;
}

} // namespace rplrio