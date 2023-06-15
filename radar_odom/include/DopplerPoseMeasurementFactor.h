#ifndef DOPPLER_POSE_MEASUREMENT_FACTOR_H
#define DOPPLER_POSE_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

class DopplerPoseMeasurementFactor : public NoiseModelFactor2<Vector3, Pose3>
{
private:
    Point3 unit_r;
    double doppler;
public:
    DopplerPoseMeasurementFactor(Key key_v, Key key_p, float x, float y, float z, float v, const SharedNoiseModel &model);

    ~DopplerPoseMeasurementFactor();

    Vector evaluateError(const Vector3 &vel, const Pose3 &pose,
                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const;
};

#endif