#ifndef VELOCITY_POSE_EXTRINSIC_FACTOR_H
#define VELOCITY_POSE_EXTRINSIC_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace std;
using namespace gtsam;

class VelocityPoseExtrinsicFactor : public NoiseModelFactor3<Vector3, Pose3, Rot3>
{
private:
    Vector3 v_measure;

public:
    VelocityPoseExtrinsicFactor(Key v_key, Key p_key, Key e_key, Vector3 measure, const SharedNoiseModel &model);

    ~VelocityPoseExtrinsicFactor();

    Vector evaluateError(const Vector3 &vel, const Pose3 &pose, const Rot3 &extrinsic,
                         boost::optional<Matrix &> H1, boost::optional<Matrix &> H2, boost::optional<Matrix &> He) const;
};

#endif