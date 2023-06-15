#include "DopplerMeasurementFactor.h"

DopplerMeasurementFactor::DopplerMeasurementFactor(Key j, float x, float y, float z, float v,
                                                   const SharedNoiseModel &model) : NoiseModelFactor1<Vector3>(model, j)
{
    unit_r = Point3(x, y, z);
    unit_r = -unit_r / unit_r.norm();
    doppler = v;
}

gtsam::Vector DopplerMeasurementFactor::evaluateError(const gtsam::Vector3 &vel,
                                                      boost::optional<gtsam::Matrix &> H = boost::none) const
{
    double err = unit_r.dot(vel) - doppler;
    if (H)
        *H = (Matrix(1, 3) << unit_r(0), unit_r(1), unit_r(2)).finished();
    return Vector1(err);
}
 
DopplerMeasurementFactor::~DopplerMeasurementFactor() {}