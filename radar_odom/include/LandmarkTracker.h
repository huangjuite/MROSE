#ifndef LANDMARKTRACKER_H
#define LANDMARKTRACKER_H

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <gtsam/geometry/Pose3.h>

using namespace std;

struct RadarKeypoint
{
    uint id;                      // id
    uint alive;                   // traked frame
    gtsam::Vector3 p;             // current position
    gtsam::Vector3 p_;            // previous posiion
    pcl::FPFHSignature33 feature; // FPFH of point closet to center
    gtsam::Matrix33 cov;
    gtsam::Matrix33 cov_;
};

class LandmarkTracker
{
private:
    gtsam::Pose3 previous_pose;

    deque<pair<gtsam::Pose3, pcl::PointCloud<pcl::PointXYZI>>> cache;
    pcl::PassThrough<pcl::PointXYZI> snr_filter;

    // pcl::search::KdTree<pcl::PointXYZI>::Ptr ec_tree;
    // EuclideanClusterExtraction<PointXYZI> ec;
    pcl::ConditionalEuclideanClustering<pcl::PointXYZI> cec;

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr ne_tree;
    pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr pf_tree;

    pcl::PointCloud<pcl::PointXYZI>::Ptr segment;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_seg;

    vector<RadarKeypoint> tracked_landmark, untracked_landmark;

    uint id_tracked = 1;
    uint step = 0;
    uint frame_stack = 4;

    float feature_distance(float *a, float *b);

public:
    LandmarkTracker(/* args */);
    ~LandmarkTracker();

    bool update(gtsam::Pose3 pose, pcl::PointCloud<pcl::PointXYZI>::ConstPtr points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr get_segment();
    pcl::PointCloud<pcl::PointXYZI>::Ptr get_cluster();
    vector<RadarKeypoint> get_tracked();
};

#endif