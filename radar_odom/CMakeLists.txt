cmake_minimum_required(VERSION 3.10.2)
project(radar_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ndt_omp
  pcl_radar
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)

# list(APPEND CMAKE_PREFIX_PATH "/home/rpl/package/libtorch")
# find_package(Torch REQUIRED)


add_service_files(
  FILES
  Registration.srv
  Prediction.srv
)

generate_dynamic_reconfigure_options(
    cfg/mapping.cfg
    cfg/extrinsic.cfg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ndt_omp
  pcl_radar
  octomap_ros
  octomap_msgs
)

add_library(${PROJECT_NAME} SHARED
  src/rplrio_types.cpp
  src/params.cpp

  src/VelocityPoseMeasurementFactor.cpp

  src/RadarVelocityEstimatorCeres.cpp
  src/RadarVelocityEstimatorEigenWeighted.cpp

  src/MapOptimize.cpp

  Scancontext/Scancontext.cpp

  src/StaticInitializer.cpp

  src/DIOManager.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  Scancontext
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  gtsam
  gtsam_unstable 
  ndt_omp
  Ceres::ceres
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(velocity_cov app/velocity_cov.cpp )
target_link_libraries(velocity_cov ${PROJECT_NAME})

add_executable(doppler_odom_bag app/DIOrunbag.cpp )
target_link_libraries(doppler_odom_bag ${PROJECT_NAME})

add_executable(publish_gt app/pub_gt_path.cpp)
target_link_libraries(publish_gt ${PROJECT_NAME})

add_executable(vo_ex app/extrinsicEstimation_velocity.cpp)
add_dependencies(vo_ex ${catkin_EXPORTED_TARGETS})
target_link_libraries(vo_ex ${PROJECT_NAME})

add_executable(doppler_odom app/DIO.cpp)
add_dependencies(doppler_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(doppler_odom ${PROJECT_NAME})

add_executable(map_optimize app/map_node.cpp)
target_link_libraries(map_optimize ${PROJECT_NAME})

add_executable(map_optimize_weighted app/map_node_weighted.cpp)
target_link_libraries(map_optimize_weighted ${PROJECT_NAME})
