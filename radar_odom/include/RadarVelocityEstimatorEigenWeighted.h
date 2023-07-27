// PCL includes
#include "RadarVelocityEstimatorBase.h"
#include <Eigen/Dense>

class RadarVelocityEstimatorEigenWeighted : public RadarVelocityEstimatorBase<pcl::PointXYZISW>
{

public:
    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
};
