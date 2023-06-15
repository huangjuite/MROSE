#include "DopplerPoseMeasurementFactor.h"

DopplerPoseMeasurementFactor::DopplerPoseMeasurementFactor(
    Key key_v, Key key_p, float x, float y, float z, float v,
    const SharedNoiseModel &model) : NoiseModelFactor2<Vector3, Pose3>(model, key_v, key_p)
{
    unit_r = Point3(x, y, z);
    unit_r = -unit_r / unit_r.norm();
    doppler = v;
}

gtsam::Vector DopplerPoseMeasurementFactor::evaluateError(const Vector3 &vel, const Pose3 &pose,
                                                          boost::optional<Matrix &> H1, boost::optional<Matrix &> H2) const
{
    
    auto pmatrix = pose.rotation().matrix();
    double err = unit_r.dot(vel) - doppler;

    if (H1)
    {
        *H1 = (Matrix(1, 3) << pmatrix.row(0).dot(unit_r),
               pmatrix.row(1).dot(unit_r),
               pmatrix.row(2).dot(unit_r))
                  .finished();
    }
    if (H2)
    {
        *H2 = Matrix::Zero(1, 6);
        Matrix13 jacob;
        jacob << unit_r(0) * vel(0), unit_r(1) * vel(0), unit_r(2) * vel(0),
            unit_r(0) * vel(1), unit_r(1) * vel(1), unit_r(2) * vel(1),
            unit_r(0) * vel(2), unit_r(1) * vel(2), unit_r(2) * vel(2);

        H2->block(0, 0, 0, 3) = jacob;
    }
    return Vector1(err);
}

DopplerPoseMeasurementFactor::~DopplerPoseMeasurementFactor() {}