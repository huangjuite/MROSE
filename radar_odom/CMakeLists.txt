cmake_minimum_required(VERSION 3.10.2)
project(radar_odom)

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
add_library(rplrio SHARED
  # src/radar_point_type.cpp
  src/rplrio_types.cpp
  src/params.cpp
  src/utils.cpp

  src/VelocityPoseMeasurementFactor.cpp

  src/RadarVelocityEstimatorCeres.cpp

  src/StaticInitializer.cpp

  src/DIOManager.cpp
  src/DIORosNode.cpp
  src/MapRosNode.cpp
  # src/RIOManager.cpp

  # src/MapOptimize.cpp
  # src/MapConstraint.cpp

)

target_include_directories(rplrio PUBLIC
  include
)

target_link_libraries(rplrio
  Eigen3::Eigen
  gtsam
  ceres
  yaml-cpp
  OpenMP::OpenMP_CXX
)

ament_target_dependencies(rplrio
  rclcpp
  pcl_ros
  visualization_msgs
  nav_msgs
)

# Build executables
#############################################

add_executable(runDIO app/runDIO.cpp)
target_link_libraries(runDIO rplrio)

add_executable(runMapping app/runMapping.cpp)
target_link_libraries(runMapping rplrio)

# add_executable(rio app/RIO.cpp)
# target_link_libraries(rio rplrio)

# add_executable(rio_map app/RIO_map.cpp)
# target_link_libraries(rio_map rplrio)



# Install
#############################################
install(
  TARGETS rplrio
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
