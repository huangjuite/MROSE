#ifndef VELOCITY_POSE_EXTRINSIC_FACTOR_H
#define VELOCITY_POSE_EXTRINSIC_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

class VelocityPoseExtrinsicFactor : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Pose3, gtsam::Rot3>
{
private:
    gtsam::Vector3 v_measure;

public:
    VelocityPoseExtrinsicFactor(gtsam::Key v_key, gtsam::Key p_key, gtsam::Key e_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model);

    ~VelocityPoseExtrinsicFactor();

    gtsam::Vector evaluateError(const gtsam::Vector3 &vel, const gtsam::Pose3 &pose, const gtsam::Rot3 &extrinsic,
                         boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2, boost::optional<gtsam::Matrix &> He) const;
};

#endif