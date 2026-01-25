cmake_minimum_required(VERSION 3.10.2)
project(rpl_rio)

add_compile_options(-std=c++17)

# Enable compile commands for clangd in VSCode
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
#############################################
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

# nanobind setup
find_package(Python REQUIRED COMPONENTS Interpreter Development)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR
  RESULT_VARIABLE NB_RESULT)
if(NB_RESULT OR NOT NB_DIR)
  message(STATUS "nanobind --cmake_dir failed, using fallback path")
  set(NB_DIR "/home/rpldocker/.local/lib/python3.10/site-packages/nanobind/cmake")
endif()
message(STATUS "nanobind cmake dir: ${NB_DIR}")
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
# Set position-independent code flag for rerun_sdk
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
FetchContent_MakeAvailable(rerun_sdk)

# Build the library
#############################################
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

# Python bindings with nanobind
#############################################
nanobind_add_module(_core python/bindings.cpp)
target_link_libraries(_core PRIVATE rio)
target_include_directories(_core PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Install Python package
ament_python_install_package(rpl_rio PACKAGE_DIR python/rpl_rio)

# Install nanobind extension into the Python package
install(TARGETS _core
  LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/rpl_rio"
)

# Install
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
