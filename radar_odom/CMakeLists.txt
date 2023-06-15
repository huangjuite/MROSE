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

find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(Eigen REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)


add_service_files(
  FILES
  Registration.srv
)

generate_dynamic_reconfigure_options(
    cfg/mapping.cfg
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
  src/DopplerMeasurementFactor.cpp
  src/DopplerCelestialFactor.cpp
  src/DopplerCelestial2Factor.cpp
  src/DopplerPoseMeasurementFactor.cpp
  src/VelocityMeasurementFactor.cpp
  src/VelocityPoseMeasurementFactor.cpp
  src/VelocityPoseExtrinsicFactor.cpp
  src/RadarVelocityEstimator.cpp
  src/RadarVelocityEstimatorGNC.cpp
  src/RadarVelocityEstimatorHCS.cpp
  src/RadarVelocityEstimatorCelestial.cpp
  src/RadarVelocityEstimatorCeres.cpp
  src/RadarVelocityEstimatorCeresHCS.cpp
  src/LandmarkFactor.cpp
  src/LandmarkTracker.cpp
  src/StaticInitializer.cpp
  src/utils.cpp
  Scancontext/Scancontext.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  include
  Scancontext
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# target_link_directories(${PROJECT_NAME}
#   PUBLIC
#   ${PCL_LIBRARY_DIRS}
# )

# target_compile_definitions(${PROJECT_NAME}
#   PUBLIC
#   ${PCL_DEFINITIONS}
# )

target_link_libraries(${PROJECT_NAME}
  gtsam
  gtsam_unstable 
  ndt_omp
  Ceres::ceres
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_executable(ImuFactorsExample app/ImuFactorsExample.cpp)
target_link_libraries(ImuFactorsExample ${PROJECT_NAME})

add_executable(ImuFactorsExample2 app/ImuFactorsExample2.cpp)
target_link_libraries(ImuFactorsExample2 ${PROJECT_NAME})

add_executable(velocityEstimation app/velocityEstimation.cpp)
target_link_libraries(velocityEstimation ${PROJECT_NAME})

add_executable(velocityEstimationCeres app/velocityEstimationCeres.cpp)
target_link_libraries(velocityEstimationCeres ${PROJECT_NAME})

add_executable(velocityEstimationCelestial app/velocityEstimationCelestial.cpp)
target_link_libraries(velocityEstimationCelestial ${PROJECT_NAME})

add_executable(test_clustering app/test_clustering.cpp )
target_link_libraries(test_clustering ${PROJECT_NAME})

# add_executable(radar_icp app/radar_icp.cpp)
# target_link_libraries(radar_icp ${catkin_LIBRARIES} ${PROJECT_NAME} gtsam)

# add_executable(radar_ndt app/radar_ndt.cpp)
# target_link_libraries(radar_ndt ${catkin_LIBRARIES} ${PROJECT_NAME} gtsam)

# add_executable(radar_ndt_omp app/radar_ndt_omp.cpp)
# target_link_libraries(radar_ndt_omp ${catkin_LIBRARIES} ${PROJECT_NAME} gtsam ndt_omp)

add_executable(dio_ex app/extrinsicEstimation.cpp)
add_dependencies(dio_ex ${catkin_EXPORTED_TARGETS})
target_link_libraries(dio_ex ${PROJECT_NAME})

add_executable(doppler_odom src/DopplerInertialOdom.cpp)
add_dependencies(doppler_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(doppler_odom ${PROJECT_NAME})

add_executable(doppler_landmark_odom src/DopplerInertialLandmarkOdom.cpp)
add_dependencies(doppler_landmark_odom ${catkin_EXPORTED_TARGETS})
target_link_libraries(doppler_landmark_odom ${PROJECT_NAME})

add_executable(doppler_odom_multi src/DopplerInertialOdomMulti.cpp)
add_dependencies(doppler_odom_multi ${catkin_EXPORTED_TARGETS})
target_link_libraries(doppler_odom_multi ${PROJECT_NAME})

# add_executable(rio src/RadarInertialOdom.cpp)
# add_dependencies(rio ${catkin_EXPORTED_TARGETS})
# target_link_libraries(rio ${PROJECT_NAME})

add_executable(radar_pose_graph src/RadarPoseGraph.cpp)
add_dependencies(radar_pose_graph ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_pose_graph ${PROJECT_NAME})

add_executable(radar_pose_graph_2d src/RadarPoseGraph2d.cpp)
add_dependencies(radar_pose_graph_2d ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_pose_graph_2d ${PROJECT_NAME})

add_executable(radar_pose_graph_2d_fgr src/RadarPoseGraph2dFGR.cpp)
add_dependencies(radar_pose_graph_2d_fgr ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_pose_graph_2d_fgr ${PROJECT_NAME})

add_executable(radar_pose_graph_2d_detect src/RadarPoseGraph2dDetect.cpp)
add_dependencies(radar_pose_graph_2d_detect ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_pose_graph_2d_detect ${PROJECT_NAME})

add_executable(radar_mapping src/RadarMapping.cpp)
add_dependencies(radar_mapping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radar_mapping ${PROJECT_NAME})

add_executable(radar_octo_mapping src/RadarOctoMapping.cpp)
add_dependencies(radar_octo_mapping ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(radar_octo_mapping  ${PROJECT_NAME})
