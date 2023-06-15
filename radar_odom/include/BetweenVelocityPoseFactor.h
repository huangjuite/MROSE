#ifndef VELOCITY_POSE_MEASUREMENT_FACTOR_H
#define VELOCITY_POSE_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

class BetweenVelocityPoseFactor : public NoiseModelFactor4<Vector3, Pose3, Vector3, Pose3>
{
private:
    Vector3 r_v1, r_v2;
    double doppler;

public:
    BetweenVelocityPoseFactor(Key v1_key, Key p1_key, Vector3 rv1,
                               Key v2_key, Key p2_key, Vector3 rv2,
                               const SharedNoiseModel &model)
        : NoiseModelFactor4<Vector3, Pose3, Vector3, Pose3>(model, v1_key, p1_key, v2_key, p2_key),
          r_v1(rv1), r_v2(rv2){};

    ~BetweenVelocityPoseFactor();

    Vector evaluateError(const Vector3 &v1, const Pose3 &p1, const Vector3 &v2, const Pose3 &p2,
                         boost::optional<Matrix &> Hv1, boost::optional<Matrix &> Hp1,
                         boost::optional<Matrix &> Hv2, boost::optional<Matrix &> Hp2) const override
    {
        Matrix33 Hr2, Hr1;
        Vector3 err = (p2.rotation().rotate(r_v2, Hr2) - p1.rotation().rotate(r_v1, Hr1)) - (v2 - v1);
        // if (H2)
        // {
        //     *H2 = Matrix::Zero(3, 6);
        //     H2->block(0, 0, 3, 3) = Hr;
        // }

        return err;
    };
};

#endif