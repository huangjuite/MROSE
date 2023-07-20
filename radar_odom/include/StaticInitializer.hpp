#ifndef STATIC_INITIALIZER_HPP
#define STATIC_INITIALIZER_HPP

#include <iostream>
#include <deque>
#include <memory>
#include <Eigen/Eigen>

#include "types.h"

class StaticInitializer
{
private:
    float init_window_ = 4;
    float acc_var_thresh_ = 0.5;
    std::deque<std::shared_ptr<rplrio::IMU>> imu_queue;
    Eigen::Quaterniond q_MtoI_;
    Eigen::Vector3d bias_acc_;
    Eigen::Vector3d bias_gyro_;
    
public:

    StaticInitializer(float init_window, float acc_var_thresh);
    bool init_update(std::shared_ptr<rplrio::IMU> imu);
    Eigen::Quaterniond get_rotation();
    Eigen::Vector3d get_bias_acc();
    Eigen::Vector3d get_bias_gyro();
    Eigen::Matrix3d gram_schmidt(Eigen::Vector3d gravity);
    std::deque<std::shared_ptr<rplrio::IMU>> get_imu_queue();
    double get_first_stamp();
    ~StaticInitializer();
};

#endif