cmake_minimum_required(VERSION 3.22)

# When invoked directly (cmake -S src -B build), set up as a standalone project.
# When included via add_subdirectory from another project, skip — the parent
# project already provides cmake_minimum_required, project(), and global flags.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(mrose VERSION 0.1.0)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
endif()

find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

add_library(rio
  core/params.cpp
  factor/VelocityPoseMeasurementFactor.cpp
  factor/VelocityPoseBiasFactor.cpp
  velocity/RadarVelocityEstimatorCeres.cpp
  velocity/RadarVelocityEstimatorEigen.cpp
  manager/StaticInitializer.cpp
  manager/ControledSmoother.cpp
  manager/DIOManager.cpp
)

target_include_directories(rio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rio PUBLIC
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  fmt::fmt
  spdlog::spdlog
)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND BUILD_PYTHON_BINDINGS)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../python
                   ${CMAKE_CURRENT_BINARY_DIR}/python)
endif()
