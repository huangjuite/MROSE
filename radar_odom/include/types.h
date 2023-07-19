#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
#include <pclradar/RadarPointType.h>

namespace rio
{
    struct IMU
    {
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        double stamp;

        IMU(Eigen::Vector3d a,
            Eigen::Vector3d g,
            double t)
        {
            acc = a;
            gyro = g;
            stamp = t;
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

        bool operator<(double right){
            return stamp < right;
        }

    };

    struct Radar{
        pcl::PointCloud<pcl::PointXYZIS>::Ptr pc;
        double stamp;

        Radar(pcl::PointCloud<pcl::PointXYZIS>::Ptr p, double t){
            stamp = t;
            pc = p;
        }
    };
};

#endif