
#define PCL_NO_PRECOMPILE
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclradar/RadarPointType.h>

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZIS>;
