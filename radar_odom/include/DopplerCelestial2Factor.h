#ifndef DOPPLER_MEASUREMENT_FACTOR_H
#define DOPPLER_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

class DopplerCelestial2Factor : public NoiseModelFactor2<Vector3, Vector2>
{
private:
    double d;

public:
    DopplerCelestial2Factor(Key vel_key, Key cel_key, double doppler, const SharedNoiseModel &model);

    ~DopplerCelestial2Factor();

    Vector evaluateError(const gtsam::Vector3 &vel, const gtsam::Vector2 &cel, boost::optional<gtsam::Matrix &> Hv,
                         boost::optional<gtsam::Matrix &> Hc) const;
};

#endif