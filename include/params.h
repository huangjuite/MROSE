#ifndef PARAMS_H
#define PARAMS_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>

typedef std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>  ImuParam_ptr;

namespace rplrio
{
    struct RadarParams
    {
        std::string topic;
        Eigen::Matrix4d T_r_i;
        Eigen::Matrix4d T_r_b;
        Eigen::Matrix3d R_r_b;
        double doppler_std;
        double azimuth_std;
        double elevation_std;
    };

    struct Params
    {
        // odom
        std::string imu_topic;
        double lag;
        bool bv_use_huber;
        double bv_huber_k;

        // initializer
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

        // loop close
        bool enable_loop_close;
        int loop_close_rate;
        int loop_ignore;
        int key_frame_search;
        double loop_distance_thresh;
        double process_voxel_size;
        double snr_thresh;
        int pc_reg_thread;
        

        // mapping
        double map_snr_thresh;
        double render_rate;
    };

    ImuParam_ptr imuParams(Params param);
};

#endif
