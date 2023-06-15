#ifndef VELOCITY_MEASUREMENT_FACTOR_H
#define VELOCITY_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

class VelocityMeasurementFactor : public NoiseModelFactor1<Vector3>
{
private:
    Vector3 v_measure;
    double doppler;

public:
    VelocityMeasurementFactor(Key v_key, Vector3 measure, const SharedNoiseModel &model);

    ~VelocityMeasurementFactor();

    Vector evaluateError(const Vector3 &vel, boost::optional<Matrix &> H1) const;
};

#endif