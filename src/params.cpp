#include "params.h"

namespace rplrio
{

    ImuParam_ptr imuParams(Params param) // EPSON G364
    {
        gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(param.acc_noise, 2);
        gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(param.gyro_noise, 2);
        gtsam::Matrix33 integration_error_cov = gtsam::I_3x3 * 1e-8; // error committed in integrating position from velocities
        gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(param.acc_rw, 2);
        gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(param.gyro_rw, 2);
        gtsam::Matrix66 bias_acc_omega_init = gtsam::I_6x6 * 1e-5; // error in the bias used for preintegration

        auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(param.gravity);
        // PreintegrationBase params:
        p->accelerometerCovariance = measured_acc_cov;    // acc white noise in continuous
        p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
        p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
        // PreintegrationCombinedMeasurements params:
        p->biasAccCovariance = bias_acc_cov;     // acc bias in continuous
        p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
        p->biasAccOmegaInt = bias_acc_omega_init;

        return p;
    }
}