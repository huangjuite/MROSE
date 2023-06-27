#ifndef VELOCITY_POSE_MEASUREMENT_FACTOR_H
#define VELOCITY_POSE_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

class VelocityPoseMeasurementFactor : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Pose3>
{
private:
    gtsam::Vector3 v_measure;

public:
    VelocityPoseMeasurementFactor(gtsam::Key v_key, gtsam::Key p_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model);

    ~VelocityPoseMeasurementFactor();

    gtsam::Vector evaluateError(const gtsam::Vector3 &vel, const gtsam::Pose3 &pose,
                         boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2) const;
};

#endif