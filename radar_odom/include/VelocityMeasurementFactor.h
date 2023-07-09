#ifndef VELOCITY_MEASUREMENT_FACTOR_H
#define VELOCITY_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>


class VelocityMeasurementFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
private:
    gtsam::Vector3 v_measure;
    double doppler;

public:
    VelocityMeasurementFactor(gtsam::Key v_key, gtsam::Vector3 measure, const gtsam::SharedNoiseModel &model);

    ~VelocityMeasurementFactor();

    gtsam::Vector evaluateError(const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H1) const;
};

#endif