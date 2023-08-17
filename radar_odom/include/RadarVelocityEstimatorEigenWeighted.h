// PCL includes
#include "RadarVelocityEstimatorBase.h"
#include <Eigen/Dense>

class RadarVelocityEstimatorEigenWeighted : public RadarVelocityEstimatorBase<pcl::PointXYZSDW>
{

public:
    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZSDW>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZSDW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
};
