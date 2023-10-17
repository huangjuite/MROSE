// std includes
#include <string>
#include <vector>
#include <iostream>
#include <deque>
#include <memory>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pclradar/RadarPointType.h>

#include <gtsam/geometry/Pose3.h>

#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>

#include "rplrio_types.h"

template <class PointType>
class MapConstraint
{
private:
    typedef typename pcl::PointCloud<PointType>::Ptr PclPtr;
    typedef typename pcl::PointCloud<PointType> Pcl;
    typedef std::shared_ptr<rplrio::Keyframe<PointType>> KeyFramePtr;

    pclomp::GeneralizedIterativeClosestPoint<PointType, PointType> gicp_omp;
    // pclomp::NormalDistributionsTransform<PointType, PointType> ndt_omp;

    pcl::VoxelGrid<PointType> voxel_filter;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_keyframes;
    std::vector<KeyFramePtr> subkeyframes;
    KeyFramePtr sub_keyframe;
    PclPtr map_;

    void get_submap(pcl::PointXYZ key_pos, float radius, PclPtr submap, pcl::PointCloud<pcl::PointXYZ>::Ptr subpos);

public:
    MapConstraint();
    ~MapConstraint();

    void update_sub_key_frame(KeyFramePtr keyframe);
    bool regist(PclPtr submap, PclPtr localmap, PclPtr aligned,
                pcl::PointCloud<pcl::PointXYZ>::Ptr subpos, gtsam::Pose3 &pose);

    void render_map(PclPtr map);
};
