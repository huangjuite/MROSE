#include "DopplerCelestial2Factor.h"

DopplerCelestial2Factor::DopplerCelestial2Factor(
    gtsam::Key vel_key, gtsam::Key cel_key, double doppler, const gtsam::SharedNoiseModel &model)
    : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector2>(model, vel_key, cel_key), d(doppler)
{
}

gtsam::Vector DopplerCelestial2Factor::evaluateError(
    const gtsam::Vector3 &vel, const gtsam::Vector2 &cel,
    boost::optional<gtsam::Matrix &> Hv, boost::optional<gtsam::Matrix &> Hc) const
{
    double theta = cel(0);
    double phi = cel(1);
    gtsam::Vector3 unit_r(-std::cos(phi) * std::cos(theta), -std::cos(phi) * std::sin(theta), -std::sin(phi));
    double err = unit_r.dot(vel) - d;

    if (Hv)
        *Hv = (gtsam::Matrix(1, 3) << unit_r(0), unit_r(1), unit_r(2)).finished();

    if (Hc)
        *Hc = (gtsam::Matrix(1, 2) << -std::cos(phi) * std::sin(theta) * vel(0) + std::cos(phi) * std::cos(theta) * vel(1),
               -std::sin(phi) * (std::cos(theta) * vel(0) + std::sin(theta) * vel(1)) + std::cos(phi) * vel(2))
                  .finished();

    return gtsam::Vector1(err);
}

DopplerCelestial2Factor::~DopplerCelestial2Factor() {}