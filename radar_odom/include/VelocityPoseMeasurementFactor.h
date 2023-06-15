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

class VelocityPoseMeasurementFactor : public NoiseModelFactor2<Vector3, Pose3>
{
private:
    Vector3 v_measure;

public:
    VelocityPoseMeasurementFactor(Key v_key, Key p_key, Vector3 measure, const SharedNoiseModel &model);

    ~VelocityPoseMeasurementFactor();

    Vector evaluateError(const Vector3 &vel, const Pose3 &pose,
                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const;
};

#endif