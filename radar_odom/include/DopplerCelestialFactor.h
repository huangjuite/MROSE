#ifndef DOPPLER_MEASUREMENT_FACTOR_H
#define DOPPLER_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

class DopplerCelestialFactor : public NoiseModelFactor1<Vector3>
{
private:
    double theta;
    double phi;
    double d;
public:
    DopplerCelestialFactor(Key j, double azimuth, double elevation, double doppler, const SharedNoiseModel &model);

    ~DopplerCelestialFactor();

    Vector evaluateError(const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H) const;
};

#endif