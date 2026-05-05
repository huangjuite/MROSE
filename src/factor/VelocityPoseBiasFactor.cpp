#include "factor/VelocityPoseBiasFactor.hpp"

#include <gtsam/base/Matrix.h>

VelocityPoseBiasFactor::VelocityPoseBiasFactor(Key v_key, Key x_key, Key b_key,
                                               Vector3 v_measure, Vector3 omega,
                                               Matrix44 T_s_i,
                                               const SharedNoiseModel &model)
    : NoiseModelFactor3<Vector3, Pose3, imuBias::ConstantBias>(model, v_key,
                                                               x_key, b_key),
      v_measure(v_measure), omega(omega), T_s_i(T_s_i) {}

Vector VelocityPoseBiasFactor::evaluateError(
    const Vector3 &vel, const Pose3 &pose,
    const imuBias::ConstantBias &imu_bias, OptionalMatrixType Hv,
    OptionalMatrixType Hx, OptionalMatrixType Hb) const {
  Matrix33 Hr;
  Vector3 bias_gyro = imu_bias.gyroscope();

  Vector3 v_i = pose.rotation().unrotate(vel, Hr, Hv);
  Vector3 v_rot =
      skewSymmetric(imu_bias.correctGyroscope(omega)) * T_s_i.block<3, 1>(0, 3);

  if (Hx) {
    *Hx = gtsam::Matrix::Zero(3, 6);
    (*Hx) << Hr, Z_3x3;
  }

  if (Hb) {
    *Hb = gtsam::Matrix::Zero(3, 6);
    // (*Hb) << Z_3x3, (-I_3x3 * -skewSymmetric(T_s_i.block<3, 1>(0, 3)));
  }

  return (v_i + v_rot) - v_measure;
}

VelocityPoseBiasFactor::~VelocityPoseBiasFactor() {}
