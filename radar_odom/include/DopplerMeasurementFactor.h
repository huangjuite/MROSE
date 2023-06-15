#ifndef DOPPLER_MEASUREMENT_FACTOR_H
#define DOPPLER_MEASUREMENT_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>

using namespace std;
using namespace gtsam;

class DopplerMeasurementFactor : public NoiseModelFactor1<Vector3>
{
private:
    Point3 unit_r;
    double doppler;

public:
    DopplerMeasurementFactor(Key j, float x, float y, float z, float v, const SharedNoiseModel &model);

    ~DopplerMeasurementFactor();

    Vector evaluateError(const gtsam::Vector3 &vel, boost::optional<gtsam::Matrix &> H) const;

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new DopplerMeasurementFactor(*this)));
    }
};

#endif