#pragma once

#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/navigation/ImuBias.h>

using namespace gtsam;

class VelocityPoseBiasFactor
    : public NoiseModelFactor3<Vector3, Pose3, imuBias::ConstantBias> {

private:
  Vector3 v_measure;
  Vector3 omega;
   Matrix44 T_s_i;

public:
  VelocityPoseBiasFactor(Key v_key, Key x_key, Key b_key, Vector3 v_measure,
                         Vector3 omega, Matrix44 T_s_i,
                         const SharedNoiseModel &model);

  ~VelocityPoseBiasFactor();

  Vector evaluateError(const Vector3 &vel, const Pose3 &pose,
                       const imuBias::ConstantBias &imu_bias, OptionalMatrixType Hv,
                       OptionalMatrixType Hx, OptionalMatrixType Hb) const;
};
