#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include "core/RadarPointType.hpp"
#include "core/params.hpp"
#include "core/rplrio_types.hpp"
#include "manager/DIOManager.hpp"
#include "velocity/RadarVelocityEstimatorCeres.hpp"

namespace nb = nanobind;
using namespace nb::literals;

// Convert PointCloudPtr to Nx4 numpy array (x, y, z, doppler)
nb::ndarray<nb::numpy, float, nb::ndim<2>>
cloud_to_ndarray(const rplrio::PointCloudPtr &cloud) {
  size_t n = cloud ? cloud->size() : 0;
  float *data = new float[n * 4];
  for (size_t i = 0; i < n; ++i) {
    const auto &pt = (*cloud)[i];
    data[i * 4 + 0] = pt.x;
    data[i * 4 + 1] = pt.y;
    data[i * 4 + 2] = pt.z;
    data[i * 4 + 3] = pt.doppler;
  }
  nb::capsule owner(data,
                    [](void *p) noexcept { delete[] static_cast<float *>(p); });
  size_t shape[2] = {n, 4};
  return nb::ndarray<nb::numpy, float, nb::ndim<2>>(data, 2, shape, owner);
}

NB_MODULE(_core, m) {
  m.doc() = "Python bindings for rpl_rio DIOManager";

  nb::enum_<rplrio::LogLevel>(m, "LogLevel")
      .value("TRACE", rplrio::LogLevel::TRACE)
      .value("DEBUG", rplrio::LogLevel::DEBUG)
      .value("INFO", rplrio::LogLevel::INFO)
      .value("WARN", rplrio::LogLevel::WARN)
      .value("ERROR", rplrio::LogLevel::ERROR)
      .value("CRITICAL", rplrio::LogLevel::CRITICAL);

  nb::class_<rplrio::RadarParams>(m, "RadarParams")
      .def(nb::init<>())
      .def_rw("T_r_i", &rplrio::RadarParams::T_r_i)
      .def_rw("R_r_i", &rplrio::RadarParams::R_r_i)
      .def_rw("doppler_std", &rplrio::RadarParams::doppler_std)
      .def_rw("azimuth_std", &rplrio::RadarParams::azimuth_std)
      .def_rw("elevation_std", &rplrio::RadarParams::elevation_std);

  nb::class_<rplrio::Params>(m, "Params")
      .def(nb::init<>())
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

  nb::class_<rplrio::IMU>(m, "IMU")
      .def(nb::init<>())
      .def(nb::init<Eigen::Vector3d, Eigen::Vector3d, double>(), "acc"_a,
           "gyro"_a, "stamp"_a)
      .def_rw("acc", &rplrio::IMU::acc)
      .def_rw("gyro", &rplrio::IMU::gyro)
      .def_rw("stamp", &rplrio::IMU::stamp);

  nb::class_<RadarVelocityEstimatorBase>(m, "RadarVelocityEstimatorBase")
      .def("set_initial_velocity",
           &RadarVelocityEstimatorBase::set_initial_velocity)
      .def("set_inlier_thresh", &RadarVelocityEstimatorBase::set_inlier_thresh)
      .def("set_doppler_sigma", &RadarVelocityEstimatorBase::set_doppler_sigma);

  nb::class_<RadarVelocityEstimatorCeres, RadarVelocityEstimatorBase>(
      m, "RadarVelocityEstimatorCeres")
      .def(nb::init<std::optional<ceres::Solver::Options>, std::optional<float>,
                    std::optional<float>>(),
           "options"_a = std::nullopt, "doppler_sigma"_a = 0.124f,
           "cauchy_scale"_a = 1.0f);

  nb::class_<rplrio::Keyframe>(m, "Keyframe")
      .def_ro("stamp", &rplrio::Keyframe::stamp)
      .def_ro("seq", &rplrio::Keyframe::seq)
      .def_ro("name", &rplrio::Keyframe::name)
      .def_prop_ro("position",
                   [](const rplrio::Keyframe &kf) {
                     gtsam::Point3 t = kf.state.pose().translation();
                     return Eigen::Vector3d(t.x(), t.y(), t.z());
                   })
      .def_prop_ro("rotation",
                   [](const rplrio::Keyframe &kf) {
                     auto q = kf.state.pose().rotation().toQuaternion();
                     return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
                   })
      .def_prop_ro("velocity",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Vector3d(kf.state.velocity());
                   })
      .def_prop_ro("pose_cov",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::MatrixXd(kf.pose_cov);
                   })
      .def_prop_ro("velocity_cov",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Matrix3d(kf.velocity_cov);
                   })
      .def_prop_ro(
          "v_err",
          [](const rplrio::Keyframe &kf) { return Eigen::Vector3d(kf.v_err); })
      .def_prop_ro("imu_acc",
                   [](const rplrio::Keyframe &kf) { return kf.imu.acc; })
      .def_prop_ro("imu_gyro",
                   [](const rplrio::Keyframe &kf) { return kf.imu.gyro; })
      .def_prop_ro("imu_bias_acc",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Vector3d(kf.imu_bias.accelerometer());
                   })
      .def_prop_ro("imu_bias_gyro",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Vector3d(kf.imu_bias.gyroscope());
                   })
      .def_prop_ro("imu_bias_cov",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::MatrixXd(kf.imu_bias_cov);
                   })
      .def_prop_ro("radar_velocity",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Vector3d(kf.radar_velocity);
                   })
      .def_prop_ro("radar_velocity_cov",
                   [](const rplrio::Keyframe &kf) {
                     return Eigen::Matrix3d(kf.radar_velocity_cov);
                   })
      .def_prop_ro(
          "point_cloud",
          [](const rplrio::Keyframe &kf) {
            return cloud_to_ndarray(kf.radar_points);
          },
          nb::rv_policy::move);

  nb::class_<rplrio::Radar>(m, "Radar")
      .def(nb::init<gtsam::Vector3, gtsam::Matrix33, double,
                    rplrio::RadarParams, std::string>(),
           "vel"_a, "vel_cov"_a, "stamp"_a, "params"_a, "name"_a)
      .def(
          "__init__",
          [](rplrio::Radar *self,
             nb::ndarray<float, nb::ndim<2>, nb::c_contig, nb::device::cpu>
                 points,
             double stamp, rplrio::RadarParams params, std::string name) {
            if (points.shape(1) != 4)
              throw std::invalid_argument(
                  "Expected 4 columns (x, y, z, doppler)");
            auto cloud = std::make_shared<rplrio::PointCloud>();
            size_t n = points.shape(0);
            cloud->resize(n);
            const float *data = points.data();
            for (size_t i = 0; i < n; ++i) {
              (*cloud)[i] = {data[i * 4 + 0], data[i * 4 + 1], data[i * 4 + 2],
                             data[i * 4 + 3]};
            }
            new (self) rplrio::Radar(cloud, stamp, params, name);
          },
          "points"_a, "stamp"_a, "params"_a, "name"_a)
      .def_ro("frame_id", &rplrio::Radar::frame_id)
      .def_ro("stamp", &rplrio::Radar::stamp)
      .def_prop_ro(
          "vel", [](const rplrio::Radar &r) { return Eigen::Vector3d(r.vel); })
      .def_prop_ro(
          "vel_cov",
          [](const rplrio::Radar &r) { return Eigen::Matrix3d(r.vel_cov); })
      .def_prop_ro(
          "point_cloud",
          [](const rplrio::Radar &r) { return cloud_to_ndarray(r.pc); },
          nb::rv_policy::move);

  nb::class_<rplrio::DIOManager>(m, "DIOManager")
      .def(nb::init<rplrio::Params,
                    std::shared_ptr<RadarVelocityEstimatorBase>>(),
           "params"_a, "velocity_estimator"_a)
      .def("feed_imu", &rplrio::DIOManager::feed_imu, "imu_data"_a)
      .def("feed_radar", &rplrio::DIOManager::feed_radar, "radar"_a)
      .def("get_latest_states", &rplrio::DIOManager::get_latest_states);

  m.def("get_params", &rplrio::get_params, "cfg_path"_a,
        "Load parameters from a YAML config file");
}
