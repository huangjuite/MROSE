#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

using namespace gtsam;

class VelocityPoseMeasurementFactor : public NoiseModelFactorN<Vector3, Pose3> {
private:
  Vector3 v_measure;

public:
  VelocityPoseMeasurementFactor(Key v_key, Key p_key, Vector3 measure,
                                const SharedNoiseModel &model);

  ~VelocityPoseMeasurementFactor();

  Vector evaluateError(const Vector3 &vel, const Pose3 &pose,
                       OptionalMatrixType H1, OptionalMatrixType H2) const;
};
