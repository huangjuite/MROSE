#ifndef ALTITUDE_FACTOR_H
#define ALTITUDE_FACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

class AltitudeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
private:
    double height;

public:
    AltitudeFactor(){};

    AltitudeFactor(gtsam::Key poseKey, const double &measured, const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), height(measured){};

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose, boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        if (H)
        {
            *H = gtsam::Matrix::Zero(1, 6);
            (*H)(0, 5) = -1;
        }
        return gtsam::Vector1(height - pose.z());
    }
};

#endif