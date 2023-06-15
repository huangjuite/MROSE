#include "VelocityMeasurementFactor.h"

VelocityMeasurementFactor::VelocityMeasurementFactor(
    Key v_key, Vector3 measure, const SharedNoiseModel &model)
    : NoiseModelFactor1<Vector3>(model, v_key), v_measure(measure)
{
}

gtsam::Vector VelocityMeasurementFactor::evaluateError(
    const Vector3 &vel, boost::optional<Matrix &> H1) const
{
    Vector3 err = vel - v_measure;
    if (H1)
    {
        *H1 = (Matrix(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1).finished();
    }
    return err;
}

VelocityMeasurementFactor::~VelocityMeasurementFactor() {}