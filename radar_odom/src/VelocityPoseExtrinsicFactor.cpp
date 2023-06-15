#include "VelocityPoseExtrinsicFactor.h"

VelocityPoseExtrinsicFactor::VelocityPoseExtrinsicFactor(
    Key v_key, Key p_key, Key e_key, Vector3 measure, const SharedNoiseModel &model)
    : NoiseModelFactor3<Vector3, Pose3, Rot3>(model, v_key, p_key, e_key), v_measure(measure)
{
}

gtsam::Vector VelocityPoseExtrinsicFactor::evaluateError(
    const Vector3 &vel, const Pose3 &pose, const Rot3 &extrinsic,
    boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> He) const
{
    Matrix33 Hr;
    Vector3 err = pose.rotation().unrotate(vel, Hr, H1) - extrinsic.rotate(v_measure, He);
    if (H2)
    {
        *H2 = Matrix::Zero(3, 6);
        H2->block(0, 0, 3, 3) = Hr;
    }
    return err;
}

VelocityPoseExtrinsicFactor::~VelocityPoseExtrinsicFactor() {}