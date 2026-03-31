cmake_minimum_required(VERSION 3.16)

project(rpl_rio)

add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

add_library(rio
  src/core/params.cpp

  src/factor/VelocityPoseMeasurementFactor.cpp
  src/factor/VelocityPoseBiasFactor.cpp
  src/velocity/RadarVelocityEstimatorCeres.cpp
  src/manager/StaticInitializer.cpp
  src/manager/ControledSmoother.cpp
  src/manager/DIOManager.cpp
)

target_include_directories(rio PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(rio
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  fmt::fmt
  OpenMP::OpenMP_CXX
)

option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
if(${BUILD_PYTHON_BINDINGS})
  add_subdirectory(python)
endif()

