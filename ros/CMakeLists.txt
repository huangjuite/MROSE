cmake_minimum_required(VERSION 3.16)
project(rpl_rio VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Core library — include from sibling src/ directory.
# IS_SYMLINK guard mirrors kiss-icp: if ros/ is symlinked standalone the
# relative ../src path would be wrong, so we fail loudly instead of silently.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../src" AND NOT IS_SYMLINK "${CMAKE_CURRENT_SOURCE_DIR}")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../src ${CMAKE_CURRENT_BINARY_DIR}/rio)
else()
  message(FATAL_ERROR "rpl_rio core not found — ros/ must not be a standalone symlink.")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

add_library(rio_ros SHARED
  src/utils.cpp
  src/ros_params.cpp
  src/DIORosNode.cpp
)
target_include_directories(rio_ros PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(rio_ros rio)
ament_target_dependencies(rio_ros
  rclcpp
  sensor_msgs
  pcl_ros
  visualization_msgs
  nav_msgs
)

add_executable(runDIO src/runDIO.cpp)
target_include_directories(runDIO PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(runDIO rio_ros)

install(TARGETS runDIO DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
