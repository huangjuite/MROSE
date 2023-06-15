#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>

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
};

#endif