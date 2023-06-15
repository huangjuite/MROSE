#include "DopplerCelestialFactor.h"

DopplerCelestialFactor::DopplerCelestialFactor(
    Key j, double azimuth, double elevation, double doppler, const SharedNoiseModel &model)
    : NoiseModelFactor1<Vector3>(model, j), d(doppler), theta(azimuth), phi(elevation)
{
}

gtsam::Vector DopplerCelestialFactor::evaluateError(
    const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H = boost::none) const
{
    gtsam::Vector3 unit_r(
        -std::cos(phi) * std::cos(theta), -std::cos(phi) * std::sin(theta), -std::sin(phi));
    double err = unit_r.dot(vel) - d;
    if (H)
        *H = (Matrix(1, 3) << unit_r(0), unit_r(1), unit_r(2)).finished();
    return Vector1(err);
}

DopplerCelestialFactor::~DopplerCelestialFactor() {}