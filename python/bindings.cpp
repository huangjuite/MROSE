#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/optional.h>

#include "DIOManager.h"
#include "RadarVelocityEstimatorCeres.h"
#include "RadarPointType.h"
#include "params.h"
#include "rplrio_types.h"
#include "utils.h"

namespace nb = nanobind;
using namespace nb::literals;

// Helper to convert PCL point cloud to numpy ndarray
template <typename PointType>
nb::ndarray<nb::numpy, float, nb::ndim<2>> pcl_to_ndarray(
    const typename pcl::PointCloud<PointType>::Ptr &cloud) {
    constexpr size_t cols = std::is_same_v<PointType, pcl::PointXYZD> ? 4 : 5;
    size_t n = cloud ? cloud->size() : 0;

    float* data = new float[n * cols];
    for (size_t i = 0; i < n; ++i) {
        const auto& pt = (*cloud)[i];
        data[i * cols + 0] = pt.x;
        data[i * cols + 1] = pt.y;
        data[i * cols + 2] = pt.z;
        if constexpr (std::is_same_v<PointType, pcl::PointXYZD>) {
            data[i * cols + 3] = pt.doppler;
        } else if constexpr (std::is_same_v<PointType, pcl::PointXYZSD>) {
            data[i * cols + 3] = pt.snr;
            data[i * cols + 4] = pt.doppler;
        }
    }

    nb::capsule owner(data, [](void* p) noexcept { delete[] static_cast<float*>(p); });
    size_t shape[2] = {n, cols};
    return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, 2, shape, owner);
}

template <typename PointType>
void bind_diomanager(nb::module_ &m, const char *name) {
    using DIOManagerT = rplrio::DIOManager<PointType>;
    using KeyframeT = rplrio::Keyframe<PointType>;
    using RadarT = rplrio::Radar<PointType>;
    using VelEstBase = RadarVelocityEstimatorBase<PointType>;
    using VelEstCeres = RadarVelocityEstimatorCeres<PointType>;

    std::string keyframe_name = std::string("Keyframe_") + name;
    std::string radar_name = std::string("Radar_") + name;
    std::string manager_name = std::string("DIOManager_") + name;
    std::string vel_est_base_name = std::string("RadarVelocityEstimatorBase_") + name;
    std::string vel_est_ceres_name = std::string("RadarVelocityEstimatorCeres_") + name;

    // Bind RadarVelocityEstimatorBase (abstract) with shared_ptr holder
    nb::class_<VelEstBase>(m, vel_est_base_name.c_str())
        .def("set_initial_velocity", &VelEstBase::set_initial_velocity)
        .def("set_inlier_thresh", &VelEstBase::set_inlier_thresh)
        .def("set_doppler_sigma", &VelEstBase::set_doppler_sigma);

    // Bind RadarVelocityEstimatorCeres
    nb::class_<VelEstCeres, VelEstBase>(m, vel_est_ceres_name.c_str())
        .def(nb::init<std::optional<ceres::Solver::Options>, std::optional<float>, std::optional<float>>(),
             "options"_a = std::nullopt, "doppler_sigma"_a = 0.124f, "cauchy_scale"_a = 1.0f);

    // Bind Keyframe
    nb::class_<KeyframeT>(m, keyframe_name.c_str())
        .def_ro("stamp", &KeyframeT::stamp)
        .def_ro("seq", &KeyframeT::seq)
        .def_ro("name", &KeyframeT::name)
        .def_prop_ro("position", [](const KeyframeT &kf) {
            gtsam::Point3 t = kf.state.pose().translation();
            return Eigen::Vector3d(t.x(), t.y(), t.z());
        })
        .def_prop_ro("rotation", [](const KeyframeT &kf) {
            auto q = kf.state.pose().rotation().toQuaternion();
            Eigen::Vector4d quat;
            quat << q.w(), q.x(), q.y(), q.z();
            return quat;
        })
        .def_prop_ro("velocity", [](const KeyframeT &kf) {
            return Eigen::Vector3d(kf.state.velocity());
        })
        .def_prop_ro("pose_cov", [](const KeyframeT &kf) {
            return Eigen::MatrixXd(kf.pose_cov);
        })
        .def_prop_ro("velocity_cov", [](const KeyframeT &kf) {
            return Eigen::Matrix3d(kf.velocity_cov);
        })
        .def_prop_ro("v_err", [](const KeyframeT &kf) {
            return Eigen::Vector3d(kf.v_err);
        })
        .def_prop_ro("imu_acc", [](const KeyframeT &kf) {
            return kf.imu.acc;
        })
        .def_prop_ro("imu_gyro", [](const KeyframeT &kf) {
            return kf.imu.gyro;
        })
        .def_prop_ro("imu_bias_acc", [](const KeyframeT &kf) {
            return Eigen::Vector3d(kf.imu_bias.accelerometer());
        })
        .def_prop_ro("imu_bias_gyro", [](const KeyframeT &kf) {
            return Eigen::Vector3d(kf.imu_bias.gyroscope());
        })
        .def_prop_ro("imu_bias_cov", [](const KeyframeT &kf) {
            return Eigen::MatrixXd(kf.imu_bias_cov);
        })
        .def_prop_ro("radar_velocity", [](const KeyframeT &kf) {
            return Eigen::Vector3d(kf.radar_velocity);
        })
        .def_prop_ro("radar_velocity_cov", [](const KeyframeT &kf) {
            return Eigen::Matrix3d(kf.radar_velocity_cov);
        })
        .def_prop_ro("point_cloud", [](const KeyframeT &kf) {
            return pcl_to_ndarray<PointType>(kf.radar_points);
        }, nb::rv_policy::move);

    // Bind Radar
    nb::class_<RadarT>(m, radar_name.c_str())
        .def(nb::init<gtsam::Vector3, gtsam::Matrix33, double, rplrio::RadarParams, std::string>(),
             "vel"_a, "vel_cov"_a, "stamp"_a, "params"_a, "name"_a)
        .def("__init__", [](RadarT *self, nb::ndarray<float, nb::ndim<2>, nb::c_contig, nb::device::cpu> points,
                            double stamp, rplrio::RadarParams params, std::string name) {
            auto cloud = typename pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
            size_t n = points.shape(0);
            size_t cols = points.shape(1);
            if (n > 0) {
                if constexpr (std::is_same_v<PointType, pcl::PointXYZD>) {
                    if (cols != 4) throw std::invalid_argument("Expected 4 columns (x,y,z,doppler)");
                } else if constexpr (std::is_same_v<PointType, pcl::PointXYZSD>) {
                    if (cols != 5) throw std::invalid_argument("Expected 5 columns (x,y,z,snr,doppler)");
                }
                cloud->resize(n);
                const float* data = points.data();
                for (size_t i = 0; i < n; ++i) {
                    auto& pt = (*cloud)[i];
                    pt.x = data[i * cols + 0];
                    pt.y = data[i * cols + 1];
                    pt.z = data[i * cols + 2];
                    if constexpr (std::is_same_v<PointType, pcl::PointXYZD>) {
                        pt.doppler = data[i * cols + 3];
                    } else if constexpr (std::is_same_v<PointType, pcl::PointXYZSD>) {
                        pt.snr = data[i * cols + 3];
                        pt.doppler = data[i * cols + 4];
                    }
                }
            }
            new (self) RadarT(cloud, stamp, params, name);
        }, "points"_a, "stamp"_a, "params"_a, "name"_a)
        .def_ro("frame_id", &RadarT::frame_id)
        .def_ro("stamp", &RadarT::stamp)
        .def_prop_ro("vel", [](const RadarT &r) { return Eigen::Vector3d(r.vel); })
        .def_prop_ro("vel_cov", [](const RadarT &r) { return Eigen::Matrix3d(r.vel_cov); })
        .def_prop_ro("point_cloud", [](const RadarT &r) {
            return pcl_to_ndarray<PointType>(r.pc);
        }, nb::rv_policy::move);

    

    // Bind DIOManager
    nb::class_<DIOManagerT>(m, manager_name.c_str())
        .def(nb::init<rplrio::Params, std::shared_ptr<VelEstBase>>(),
             "params"_a, "velocity_estimator"_a)
        .def("feed_imu", &DIOManagerT::feed_imu, "imu_data"_a)
        .def("feed_radar", &DIOManagerT::feed_radar, "radar"_a)
        .def("get_latest_states", &DIOManagerT::get_latest_states);
}

NB_MODULE(_core, m) {
    m.doc() = "Python bindings for rpl_rio DIOManager";

    // Bind LogLevel enum
    nb::enum_<rplrio::LogLevel>(m, "LogLevel")
        .value("TRACE", rplrio::LogLevel::TRACE)
        .value("DEBUG", rplrio::LogLevel::DEBUG)
        .value("INFO", rplrio::LogLevel::INFO)
        .value("WARN", rplrio::LogLevel::WARN)
        .value("ERROR", rplrio::LogLevel::ERROR)
        .value("CRITICAL", rplrio::LogLevel::CRITICAL);

    // Bind RadarParams
    nb::class_<rplrio::RadarParams>(m, "RadarParams")
        .def(nb::init<>())
        .def_rw("topic", &rplrio::RadarParams::topic)
        .def_rw("type", &rplrio::RadarParams::type)
        .def_rw("T_r_i", &rplrio::RadarParams::T_r_i)
        .def_rw("R_r_i", &rplrio::RadarParams::R_r_i)
        .def_rw("doppler_std", &rplrio::RadarParams::doppler_std)
        .def_rw("azimuth_std", &rplrio::RadarParams::azimuth_std)
        .def_rw("elevation_std", &rplrio::RadarParams::elevation_std);

    // Bind Params
    nb::class_<rplrio::Params>(m, "Params")
        .def(nb::init<>())
        .def_rw("imu_topic", &rplrio::Params::imu_topic)
        .def_rw("lag", &rplrio::Params::lag)
        .def_rw("bv_use_huber", &rplrio::Params::bv_use_huber)
        .def_rw("bv_huber_k", &rplrio::Params::bv_huber_k)
        .def_rw("marginalization_sigma", &rplrio::Params::marginalization_sigma)
        .def_rw("use_static_init", &rplrio::Params::use_static_init)
        .def_rw("init_window", &rplrio::Params::init_window)
        .def_rw("init_thres", &rplrio::Params::init_thres)
        .def_rw("T_i_b", &rplrio::Params::T_i_b)
        .def_rw("R_i_b", &rplrio::Params::R_i_b)
        .def_rw("acc_noise", &rplrio::Params::acc_noise)
        .def_rw("acc_rw", &rplrio::Params::acc_rw)
        .def_rw("gyro_noise", &rplrio::Params::gyro_noise)
        .def_rw("gyro_rw", &rplrio::Params::gyro_rw)
        .def_rw("gravity", &rplrio::Params::gravity)
        .def_rw("doppler_inlier_thresh", &rplrio::Params::doppler_inlier_thresh)
        .def_rw("radars", &rplrio::Params::radars)
        .def_rw("radar_list", &rplrio::Params::radar_list)
        .def_rw("log_level", &rplrio::Params::log_level);

    // Bind IMU
    nb::class_<rplrio::IMU>(m, "IMU")
        .def(nb::init<>())
        .def(nb::init<Eigen::Vector3d, Eigen::Vector3d, double>(),
             "acc"_a, "gyro"_a, "stamp"_a)
        .def_rw("acc", &rplrio::IMU::acc)
        .def_rw("gyro", &rplrio::IMU::gyro)
        .def_rw("stamp", &rplrio::IMU::stamp);

    // Bind DIOManager for each point type
    bind_diomanager<pcl::PointXYZD>(m, "XYZD");
    bind_diomanager<pcl::PointXYZSD>(m, "XYZSD");

    // Bind get_params function
    m.def("get_params", &rplrio::get_params, "cfg_path"_a,
          "Load parameters from a YAML config file");
}
