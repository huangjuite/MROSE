#include "MapConstraint.h"

template <class PointType>
MapConstraint<PointType>::MapConstraint(/* args */)
{
    cloud_keyframes.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

template <class PointType>
MapConstraint<PointType>::~MapConstraint()
{
}

template <class PointType>
void MapConstraint<PointType>::update_sub_key_frame(KeyFramePtr keyframe)
{
    if (sub_keyframe == nullptr)
    {
        sub_keyframe = std::make_shared<rplrio::Keyframe<PointType>>(*keyframe);
        PclPtr np(new pcl::PointCloud<PointType>);
        sub_keyframe->radar_points = np;
        pcl::copyPointCloud(*keyframe->radar_points, *sub_keyframe->radar_points);
    }
    else
    {
        auto prev_pose = sub_keyframe->state.pose();
        auto current_pose = keyframe->state.pose();
        auto prev_points = sub_keyframe->radar_points;
        *sub_keyframe = *keyframe;

        auto T = current_pose.between(prev_pose);
        pcl::transformPointCloud(*prev_points, *prev_points, T.matrix());
        sub_keyframe->radar_points = prev_points;
        *sub_keyframe->radar_points += *keyframe->radar_points;
    }
}

template <class PointType>
void MapConstraint<PointType>::get_submap(
    pcl::PointXYZ key_pos, float radius, PclPtr submap, pcl::PointCloud<pcl::PointXYZ>::Ptr subpos)
{

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_keyframes);
    std::vector<int> pointIdx;
    std::vector<float> pointDis;

    if (kdtree.radiusSearch(key_pos, radius, pointIdx, pointDis) > 0)
    {
        for (std::size_t i = 0; i < pointIdx.size(); i++)
        {
            auto idx = pointIdx[i];
            *submap += *subkeyframes[idx]->radar_points;
            subpos->points.emplace_back(cloud_keyframes->points[idx]);
        }
    }
}

template <class PointType>
gtsam::Pose3 MapConstraint<PointType>::regist(PclPtr submap, PclPtr localmap, pcl::PointCloud<pcl::PointXYZ>::Ptr subpos)
{
    pcl::transformPointCloud(
        *sub_keyframe->radar_points, *sub_keyframe->radar_points,
        sub_keyframe->state.pose().matrix());
    pcl::copyPointCloud(*sub_keyframe->radar_points, *localmap);

    auto t = sub_keyframe->state.pose().translation();
    pcl::PointXYZ key_pos(t.x(), t.y(), t.z());

    if (!subkeyframes.empty())
    {
        get_submap(key_pos, 8, submap, subpos);
    }

    cloud_keyframes->points.emplace_back(key_pos);
    subkeyframes.emplace_back(sub_keyframe);
    sub_keyframe = nullptr;

    gtsam::Pose3 pose_map;
    return pose_map;
}

template class MapConstraint<pcl::PointXYZSD>;
template class MapConstraint<pcl::PointXYZSDW>;
