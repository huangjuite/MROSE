
#define PCL_NO_PRECOMPILE
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>
#include <pclradar/RadarPointType.h>

template class pcl::PCLBase<pcl::PointXYZSD>;
template class pcl::PCLBase<pcl::PointXYZSDW>;
template class pcl::KdTreeFLANN<pcl::PointXYZSD>;
template class pcl::KdTreeFLANN<pcl::PointXYZSDW>;
template class pcl::search::KdTree<pcl::PointXYZSD, pcl::KdTreeFLANN<pcl::PointXYZSD, flann::L2_Simple<float>>>;
template class pcl::search::KdTree<pcl::PointXYZSDW, pcl::KdTreeFLANN<pcl::PointXYZSDW, flann::L2_Simple<float>>>;

template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZSD, pcl::PointXYZSD>;
template class pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZSDW, pcl::PointXYZSDW>;
