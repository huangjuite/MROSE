#ifndef LANDMARK_FACTOR_H
#define LANDMARK_FACTOR_H

#include <stdio.h>
#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;

class LandmarkFactor : public NoiseModelFactor2<Pose3, Point3>
{
private:
    Point3 measurement;

public:
    LandmarkFactor(Key k_p, Key k_l, Point3 measure_l, const SharedNoiseModel &model);

    ~LandmarkFactor();

    Vector evaluateError(const Pose3 &pose, const Point3 &landmark,
                         boost::optional<gtsam::Matrix &> Hp,
                         boost::optional<gtsam::Matrix &> Hl) const;
};

#endif