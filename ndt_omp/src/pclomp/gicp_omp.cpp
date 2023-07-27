
#define PCL_NO_PRECOMPILE
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclradar/RadarPointType.h>

template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZIS, pcl::PointXYZIS>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZISW, pcl::PointXYZISW>;


