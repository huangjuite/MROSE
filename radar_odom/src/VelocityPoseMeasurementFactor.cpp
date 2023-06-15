#include "VelocityPoseMeasurementFactor.h"

VelocityPoseMeasurementFactor::VelocityPoseMeasurementFactor(
    Key v_key, Key p_key, Vector3 measure, const SharedNoiseModel &model)
    : NoiseModelFactor2<Vector3, Pose3>(model, v_key, p_key), v_measure(measure)
{
}

gtsam::Vector VelocityPoseMeasurementFactor::evaluateError(
    const Vector3 &vel, const Pose3 &pose, boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
{
    Matrix33 Hr;
    Vector3 err = pose.rotation().unrotate(vel, Hr, H1) - v_measure;
    if (H2)
    {
        *H2 = Matrix::Zero(3, 6);
        H2->block(0, 0, 3, 3) = Hr;
    }

    return err;
}

VelocityPoseMeasurementFactor::~VelocityPoseMeasurementFactor() {}