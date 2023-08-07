#ifndef DOPPLER_MEASUREMENT_FACTOR_H
#define DOPPLER_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

class DopplerCelestialFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3>
{
private:
    double theta;
    double phi;
    double d;

public:
    DopplerCelestialFactor(
        gtsam::Key j, double azimuth, double elevation, double doppler,
        const gtsam::SharedNoiseModel &model);

    ~DopplerCelestialFactor();

    gtsam::Vector evaluateError(const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H) const;
};

#endif