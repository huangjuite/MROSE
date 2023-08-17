#include "LandmarkTracker.h"

bool enforceIntensitySimilarity(
    const pcl::PointXYZI &point_a, const pcl::PointXYZI &point_b, float)
{
    if (std::abs(point_a.intensity - point_b.intensity) < 1)
        return (true);
    else
        return (false);
}

LandmarkTracker::LandmarkTracker(/* args */)
{
    snr_filter.setFilterFieldName("intensity");
    snr_filter.setFilterLimits(3, FLT_MAX); // log dB

    cec.setConditionFunction(&enforceIntensitySimilarity);
    cec.setClusterTolerance(0.08);
    cec.setMinClusterSize(8);
    cec.setMaxClusterSize(100);

    ne_tree.reset(new pcl::search::KdTree<pcl::PointXYZI>());
    ne.setSearchMethod(ne_tree);
    ne.setRadiusSearch(0.04);

    pf_tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
    fpfh.setSearchMethod(pf_tree);
    fpfh.setRadiusSearch(0.1);

    // ec_tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
    // ec.setClusterTolerance(0.06);
    // ec.setMinClusterSize(10);
    // ec.setMaxClusterSize(100);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(segment);
}

bool LandmarkTracker::update(gtsam::Pose3 pose, pcl::PointCloud<pcl::PointXYZI>::ConstPtr points)
{
    cache.push_back({pose, *points});
    if (cache.size() < frame_stack)
    {
        return false;
    }

    segment.reset(new pcl::PointCloud<pcl::PointXYZI>);

    for (auto p : cache)
    {
        pcl::PointCloud<pcl::PointXYZI> pc = p.second;
        pcl::transformPointCloud(pc, pc, pose.between(p.first).matrix());
        *segment += pc;
    }
    // cache.pop_front();
    cache.clear();

    // snr_filter.setInputCloud(segment);
    // snr_filter.filter(*segment);

    // ec_tree->setInputCloud(segment);
    // vector<PointIndices> cluster_indices;
    // ec.extract(cluster_indices);

    pcl::IndicesClusters cluster_indices;
    cec.setInputCloud(segment);
    cec.segment(cluster_indices);

    // normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud(segment);
    ne.compute(*cloud_normals);

    // FPFH
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
    fpfh.setInputCloud(segment);
    fpfh.setInputNormals(cloud_normals);
    fpfh.compute(*fpfhs);

    // landmark cache
    vector<RadarKeypoint> landmarks;
    // cluster_seg.reset(new pcl::PointCloud<pcl::PointXYZI>);

    // build landmakr ==========================================================
    uint j = 0;
    for (const auto &cluster : cluster_indices)
    {
        // pcl::PointCloud<pcl::PointXYZI> cloud_cluster;

        // landmark position
        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*segment, cluster, centroid);

        // landmark covariance
        Eigen::Matrix3d cov_matrix;
        pcl::computeCovarianceMatrix(*segment, cluster, cov_matrix);

        // landmark FPFH
        float d_min = MAXFLOAT;
        size_t cidx = 0;
        for (const auto &idx : cluster.indices)
        {
            auto p = (*segment)[idx];
            float d = (gtsam::Vector3(p.x, p.y, p.z) - centroid.head<3>()).norm();
            if (d < d_min)
            {
                d_min = d;
                cidx = idx;
            }
        }
        landmarks.push_back({0, 0, centroid.head<3>(), centroid.head<3>(),
                             fpfhs->points[cidx], cov_matrix, cov_matrix});

        // *cluster_seg += cloud_cluster;
        j++;
    }

    // key points association ===============================================
    // cout << "\033c" << endl;
    vector<RadarKeypoint> new_tracked, new_untracked;
    vector<RadarKeypoint> pre_landmarks = tracked_landmark;
    pre_landmarks.insert(pre_landmarks.end(), untracked_landmark.begin(), untracked_landmark.end());
    auto t = previous_pose.between(pose);
    for (auto l : landmarks)
    {
        gtsam::Vector3 p = t * l.p;
        vector<uint> candidate;

        // find distance close
        for (uint pi = 0; pi < pre_landmarks.size(); pi++)
        {
            auto pl = pre_landmarks[pi];
            float d = (p - pl.p).norm();

            if (d < 0.2)
            {
                candidate.push_back(pi);
            }
        }

        // find feature close
        if (candidate.size() >= 1)
        {
            float d = MAXFLOAT;
            uint matched;
            // cout << "--------------------------" << endl;
            for (auto ci : candidate)
            {
                auto c = pre_landmarks[ci];
                float f_diff = feature_distance(
                    c.feature.histogram, l.feature.histogram);
                float d_diff = (p - c.p).norm();
                // printf("%f, %f\n", f_diff, d_diff);
                float diff = f_diff + d_diff;
                if (d > diff)
                {
                    d = diff;
                    matched = ci;
                }
            }

            l.id = pre_landmarks[matched].id;
            l.alive = pre_landmarks[matched].alive + 1;
            l.p_ = pre_landmarks[matched].p;
            l.cov_ = pre_landmarks[matched].cov;

            // printf("%d, ", l.id);
            pre_landmarks.erase(pre_landmarks.begin() + matched);

            if (l.id == 0)
                l.id = id_tracked++;

            // printf("%d, %d\n", l.id, l.alive);

            new_tracked.push_back(l);
        }
        else
        {
            new_untracked.push_back(l);
        }
    }
    // cout << "====================" << endl;
    // reset cache =================================================================

    tracked_landmark = new_tracked;
    untracked_landmark = new_untracked;

    previous_pose = pose;
    step++;

    return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LandmarkTracker::get_segment()
{
    return segment;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LandmarkTracker::get_cluster()
{
    return cluster_seg;
}

vector<RadarKeypoint> LandmarkTracker::get_tracked()
{
    return tracked_landmark;
}

float LandmarkTracker::feature_distance(float *a, float *b)
{
    double d = 0;
    for (size_t i = 0; i < 33; i++)
    {
        d += pow(a[i] - b[i], 2);
    }
    return float(sqrt(d)) / 33.0;
}

LandmarkTracker::~LandmarkTracker()
{
}
