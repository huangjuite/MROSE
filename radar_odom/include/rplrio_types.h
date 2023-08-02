#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pclradar/RadarPointType.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include "params.h"

namespace rplrio
{
    struct IMU
    {
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        double stamp;

        IMU() {}
        IMU(Eigen::Vector3d a,
            Eigen::Vector3d g,
            double t)
        {
            acc = a;
            gyro = g;
            stamp = t;
        }

        static bool interpolate(IMU &imu, double time, IMU imu1, IMU imu2)
        {
            if (time > imu2.stamp || time < imu1.stamp || imu1.stamp > imu2.stamp)
                return false;

            double ratio = (time - imu1.stamp) / (imu2.stamp - imu1.stamp);
            imu.stamp = ratio * (imu2.stamp - imu1.stamp) + imu1.stamp;
            imu.acc = ratio * (imu2.acc - imu1.acc) + imu1.acc;
            imu.gyro = ratio * (imu2.gyro - imu1.gyro) + imu1.gyro;

            return true;
        }
    };

    struct Odom
    {
        double stamp;
        uint seq;
        gtsam::Pose3 pose;
        Eigen::Vector3d v;
        Eigen::Vector3d a;

        Odom(double t,
             uint s,
             gtsam::Pose3 p,
             Eigen::Vector3d lv,
             Eigen::Vector3d av)
        {
            stamp = t;
            seq = s;
            pose = p;
            v = lv;
            a = av;
        }

        bool operator<(double right)
        {
            return stamp < right;
        }
    };

    template <class PointType>
    struct Radar
    {
        typedef typename pcl::PointCloud<PointType>::Ptr myPcPtr;
        myPcPtr pc;
        double stamp;
        RadarParams param;

        Radar(myPcPtr p, double t, RadarParams params)
        {
            stamp = t;
            pc = p;
            param = params;
        }
    };

    template <class PointType>
    struct Keyframe
    {
        double stamp;
        uint seq;
        gtsam::NavState state;
        gtsam::Matrix pose_cov;

        gtsam::imuBias::ConstantBias imu_bias;

        typedef typename pcl::PointCloud<PointType>::Ptr myPcPtr;
        myPcPtr radar_points;
        gtsam::Vector3 radar_velocity;
        gtsam::Matrix3 radar_velocity_cov;
    };

};

#endif