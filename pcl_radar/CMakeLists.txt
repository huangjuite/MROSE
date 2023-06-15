cmake_minimum_required(VERSION 3.0.2)
project(pcl_radar)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
)


find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pcl_radar
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_library(pcl_radar SHARED
  src/pclradar/radar_point_type.cpp
)


install(
  TARGETS
    pcl_radar
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install headers
install(DIRECTORY include/pclradar
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})


