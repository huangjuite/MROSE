#ifndef PARAMS_H
#define PARAMS_H

#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <string>
#include <vector>

typedef std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>
    ImuParam_ptr;

namespace rplrio {
struct RadarParams {
  std::string topic;
  Eigen::Matrix4d T_r_i;
  Eigen::Matrix3d R_r_i;
  double doppler_std;
  double azimuth_std;
  double elevation_std;
};

struct Params {
  // odom
  std::string imu_topic;
  double lag;
  bool bv_use_huber;
  double bv_huber_k;

  // initializer
  bool use_static_init;
  double init_window;
  double init_thres;

  // imu
  Eigen::Matrix4d T_i_b;
  Eigen::Matrix3d R_i_b;
  double acc_noise;
  double acc_rw;
  double gyro_noise;
  double gyro_rw;
  double gravity;

  // radar
  double doppler_inlier_thresh;
  std::unordered_map<std::string, RadarParams> radars;
  std::vector<std::string> radar_list;
  float target_frame_rate;

  // mapping
  double process_voxel_size;
  double map_snr_thresh;
  double render_rate;
};

ImuParam_ptr imuParams(Params param);
}; // namespace rplrio

#endif
