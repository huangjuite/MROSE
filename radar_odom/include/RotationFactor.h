#ifndef ROTATION_FACTOR_H
#define ROTATION_FACTOR_H

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

using namespace gtsam;

class RotationFactor : public NoiseModelFactor1<Pose3>
{
private:
    double pitch;
    double roll;

public:
    RotationFactor(){};

    RotationFactor(Key poseKey, const double &p, const double &r, const SharedNoiseModel &model)
        : NoiseModelFactor1<Pose3>(model, poseKey), pitch(p), roll(r){};

    Vector evaluateError(const Pose3 &pose, boost::optional<Matrix &> H = boost::none) const override
    {
        Matrix13 Hr, Hp;
        auto err = Vector2(pose.rotation().roll(Hr) - roll, pose.rotation().pitch(Hp) - pitch);
        if (H)
        {
            *H = Matrix::Zero(2, 6);
            H->block(0, 0, 1, 3) = Hr;
            H->block(1, 0, 1, 3) = Hp;
        }
        return err;
    }
};

#endif