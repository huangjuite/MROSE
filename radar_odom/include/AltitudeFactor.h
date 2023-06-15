#ifndef ALTITUDE_FACTOR_H
#define ALTITUDE_FACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

class AltitudeFactor : public NoiseModelFactor1<Pose3>
{
private:
    double height;

public:
    AltitudeFactor(){};

    AltitudeFactor(Key poseKey, const double &measured, const SharedNoiseModel &model)
        : NoiseModelFactor1<Pose3>(model, poseKey), height(measured){};

    Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const override
    {
        if (H)
        {
            *H = Matrix::Zero(1, 6);
            (*H)(0, 5) = -1;
        }
        return Vector1(height - pose.z());
    }
};

#endif