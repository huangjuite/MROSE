#include "VelocityPoseMeasurementFactor.h"

VelocityPoseMeasurementFactor::VelocityPoseMeasurementFactor(
    gtsam::Key v_key, gtsam::Key p_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor2<gtsam::Vector3, gtsam::Pose3>(model, v_key, p_key), v_measure(measure)
{
}

gtsam::Vector VelocityPoseMeasurementFactor::evaluateError(
    const gtsam::Vector3 &vel, const gtsam::Pose3 &pose, 
    boost::optional<gtsam::Matrix &> H1, 
    boost::optional<gtsam::Matrix &> H2) const
{
    gtsam::Matrix33 Hr;
    gtsam::Vector3 err = pose.rotation().unrotate(vel, Hr, H1) - v_measure;
    if (H2)
    {
        *H2 = gtsam::Matrix::Zero(3, 6);
        H2->block(0, 0, 3, 3) = Hr;
    }

    return err;
}

VelocityPoseMeasurementFactor::~VelocityPoseMeasurementFactor() {}