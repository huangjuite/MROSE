#include "LandmarkFactor.h"

LandmarkFactor::LandmarkFactor(
    Key k_p, Key k_l, Point3 measure_l, const SharedNoiseModel &model)
    : NoiseModelFactor2<Pose3, Point3>(model, k_p, k_l), measurement(measure_l)
{
}

gtsam::Vector LandmarkFactor::evaluateError(
    const Pose3 &pose, const Point3 &landmark,
    boost::optional<gtsam::Matrix &> Hp,
    boost::optional<gtsam::Matrix &> Hl) const
{
    Vector3 err = pose.transformTo(landmark, Hp, Hl) - measurement;
    return err;
}

LandmarkFactor::~LandmarkFactor() {}