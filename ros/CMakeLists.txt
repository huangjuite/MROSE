cmake_minimum_required(VERSION 3.16)

project(rpl_rio_ros)

add_compile_options(-std=c++17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Core library (ROS-independent)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/.. ${CMAKE_CURRENT_BINARY_DIR}/rpl_rio_core)

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

add_library(rio_ros SHARED
  utils.cpp
  DIORosNode.cpp
)
target_include_directories(rio_ros PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rio_ros rio)
ament_target_dependencies(rio_ros
  rclcpp
  sensor_msgs
  pcl_ros
  visualization_msgs
  nav_msgs
)

add_executable(runDIO runDIO.cpp)
target_include_directories(runDIO PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(runDIO rio_ros)

install(TARGETS runDIO DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
