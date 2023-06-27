#include "VelocityPoseExtrinsicFactor.h"

VelocityPoseExtrinsicFactor::VelocityPoseExtrinsicFactor(
    gtsam::Key v_key, gtsam::Key p_key, gtsam::Key e_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model)
    : NoiseModelFactor3<gtsam::Vector3, gtsam::Pose3, gtsam::Rot3>(model, v_key, p_key, e_key), v_measure(measure)
{
}

gtsam::Vector VelocityPoseExtrinsicFactor::evaluateError(
    const gtsam::Vector3 &vel, const gtsam::Pose3 &pose, const gtsam::Rot3 &extrinsic,
    boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, boost::optional<gtsam::Matrix &> He) const
{
    gtsam::Matrix33 Hr;
    gtsam::Vector3 err = pose.rotation().unrotate(vel, Hr, H1) - extrinsic.rotate(v_measure, He);
    if (H2)
    {
        *H2 = gtsam::Matrix::Zero(3, 6);
        H2->block(0, 0, 3, 3) = Hr;
    }
    return err;
}

VelocityPoseExtrinsicFactor::~VelocityPoseExtrinsicFactor() {}