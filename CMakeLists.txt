cmake_minimum_required(VERSION 3.22)

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

project(rpl_rio)

add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(form REQUIRED)

include(FetchContent)
FetchContent_Declare(rerun_sdk
  URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(rerun_sdk)

# Library
add_library(rio SHARED
  src/params.cpp
  src/utils.cpp

  src/VelocityPoseMeasurementFactor.cpp
  src/VelocityPoseBiasFactor.cpp
  src/RadarVelocityEstimatorCeres.cpp
  src/StaticInitializer.cpp
  src/ControledSmoother.cpp
  src/DIOManager.cpp

  src/RosNode/DIORosNode.cpp
  src/RosNode/RerunLogger.cpp

  src/form/RadarFeatureExtractor.cpp
  src/form/formRadar.cpp

)

target_include_directories(rio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(rio
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  fmt::fmt
  OpenMP::OpenMP_CXX
  rerun_sdk
  form::FORM
)

ament_target_dependencies(rio
  rclcpp
  sensor_msgs
  pcl_ros
  visualization_msgs
  nav_msgs
)

# Build executables
#############################################

add_executable(runDIO app/runDIO.cpp)
target_link_libraries(runDIO rio)

add_executable(bagDIO app/bagDIO.cpp)
target_link_libraries(bagDIO rio)
ament_target_dependencies(bagDIO
  rosbag2_cpp
  rosbag2_storage
)

add_executable(compRIO app/compRIO.cpp)
target_link_libraries(compRIO rio)
ament_target_dependencies(compRIO
  rosbag2_cpp
  rosbag2_storage
)

add_executable(runform app/runform.cpp)
target_link_libraries(runform rio)
ament_target_dependencies(runform
  rosbag2_cpp
  rosbag2_storage
)

# Python bindings
ament_python_install_package(rpl_rio PACKAGE_DIR python/rpl_rio)
add_subdirectory(python)


#############################################
# Install and export the main library target
install(
  TARGETS rio
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install rerun_sdk so it's available to downstream packages
install(
  TARGETS rerun_sdk
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  runDIO
  bagDIO
  compRIO
  runform
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export targets for downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export dependencies so downstream packages can find them
ament_export_dependencies(
  Eigen3
  GTSAM
  Ceres
  yaml-cpp
  fmt
  rclcpp
  sensor_msgs
  pcl_ros
  visualization_msgs
  nav_msgs
)

# Export include directories
ament_export_include_directories(include)

# Export libraries
ament_export_libraries(rio)

ament_package()
