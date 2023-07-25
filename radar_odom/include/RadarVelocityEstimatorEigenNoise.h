// PCL includes
#include <stdlib.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pclradar/RadarPointType.h>

#include <Eigen/Dense>


class RadarVelocityEstimatorEigenNoise
{
private:
    Eigen::Vector3d initial_v;
    Eigen::Matrix3d covariance_matrix;
    pcl::PointCloud<pcl::PointXYZISW>::Ptr inlier_pc;
    float inlier_thresh;

    float doppler_projection(pcl::PointXYZISW p, Eigen::Vector3d v);

public:
    RadarVelocityEstimatorEigenNoise();
    ~RadarVelocityEstimatorEigenNoise();

    Eigen::Vector3d optimize(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc);
    Eigen::Matrix3d get_covariance(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    pcl::PointCloud<pcl::PointXYZISW>::Ptr get_inlier(pcl::PointCloud<pcl::PointXYZISW>::ConstPtr radar_pc, Eigen::Vector3d velocity);
    void set_initial_velocity(Eigen::Vector3d init_v);
    void set_inlier_thresh(float thresh);
};
