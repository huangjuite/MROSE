
#define PCL_NO_PRECOMPILE
#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclradar/RadarPointType.h>

template class pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZIS, pcl::PointXYZIS>;
template class pclomp::NormalDistributionsTransform<pcl::PointXYZISW, pcl::PointXYZISW>;

