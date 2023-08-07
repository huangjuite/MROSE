#ifndef DOPPLER_MEASUREMENT_FACTOR_H
#define DOPPLER_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

class DopplerCelestial2Factor : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector2>
{
private:
    double d;

public:
    DopplerCelestial2Factor(
        gtsam::Key vel_key, gtsam::Key cel_key, double doppler, const gtsam::SharedNoiseModel &model);

    ~DopplerCelestial2Factor();

    gtsam::Vector evaluateError(const gtsam::Vector3 &vel, const gtsam::Vector2 &cel,
                         boost::optional<gtsam::Matrix &> Hv, boost::optional<gtsam::Matrix &> Hc) const;
};

#endif