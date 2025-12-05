cmake_minimum_required(VERSION 3.10.2)
project(rpl_rio)

add_compile_options(-std=c++17)

# Enable compile commands for clangd in VSCode
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
#############################################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

# ROS messages and services
#############################################
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# Build the library
#############################################
add_library(rio SHARED
  src/params.cpp
  src/utils.cpp

  src/VelocityPoseMeasurementFactor.cpp
  src/RadarVelocityEstimatorCeres.cpp
  src/StaticInitializer.cpp
  src/ControledSmoother.cpp
  src/DIOManager.cpp

  src/RosNode/DIORosNode.cpp
  src/RosNode/RerunLogger.cpp

)

target_include_directories(rio PUBLIC
  include
)

target_link_libraries(rio
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  fmt::fmt
  OpenMP::OpenMP_CXX
  rerun_sdk
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

add_executable(test_velocity_estimator app/test_vest.cpp)
target_link_libraries(test_velocity_estimator rio)

add_executable(bagDIO app/bagDIO.cpp)
target_link_libraries(bagDIO rio)
ament_target_dependencies(bagDIO
  rosbag2_cpp
  rosbag2_storage
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bagDIO "${cpp_typesupport_target}")

# Install
#############################################
install(
  TARGETS rio
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  runDIO
  test_velocity_estimator
  bagDIO
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

ament_package()
