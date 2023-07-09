#include "VelocityMeasurementFactor.h"

VelocityMeasurementFactor::VelocityMeasurementFactor(
    gtsam::Key v_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor1<gtsam::Vector3>(model, v_key), v_measure(measure)
{
}

gtsam::Vector VelocityMeasurementFactor::evaluateError(
    const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H1) const
{
    gtsam::Vector3 err = vel - v_measure;
    if (H1)
    {
        *H1 = gtsam::Matrix33::Identity();
    }
    return err;
}

VelocityMeasurementFactor::~VelocityMeasurementFactor() {}