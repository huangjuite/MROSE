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


# ROS messages and services
#############################################
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  "srv/GetMap.srv"
  "srv/Registration.srv"
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

  src/DIOManager.cpp

  src/RosNode/DIORosNode.cpp
  src/RosNode/MapRosNode.cpp

)

target_include_directories(rio PUBLIC
  include
)

target_link_libraries(rio
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(rio
  rclcpp
  pcl_ros
  visualization_msgs
  nav_msgs
)

# Build executables
#############################################

add_executable(runDIO app/runDIO.cpp)
target_link_libraries(runDIO rio)

add_executable(runMapping app/runMapping.cpp)
target_link_libraries(runMapping rio)

add_executable(test_velocity_estimator app/test_vest.cpp)
target_link_libraries(test_velocity_estimator rio)

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
  runMapping
  test_velocity_estimator
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
