#include "RadarMapping.h"

RadarMapping::RadarMapping(NodeHandle *nh)
{
    param::get("~process_voxel_size", process_voxel_size);

    full_map.reset(new pcl::PointCloud<pcl::PointXYZSD>);
    voxel_filter.setLeafSize(process_voxel_size, process_voxel_size, process_voxel_size);
    snr_filter.setFilterFieldName("snr");
    snr_filter.setFilterLimits(snr_thresh, FLT_MAX);

    pub_map = nh->advertise<sensor_msgs::PointCloud2>("radar_map", 1);
    sub_loop = nh->subscribe<visualization_msgs::Marker>("loop_detected", 1, &RadarMapping::cb_loop, this);
    sub_radar.subscribe(*nh, "radar_inlier", 20);
    sub_path.subscribe(*nh, "radar_path", 20);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), sub_radar, sub_path);
    sync->registerCallback(boost::bind(&RadarMapping::cb_node, this, _1, _2));

    f = boost::bind(&RadarMapping::cb_config, this, _1, _2);
    server.setCallback(f);

    ROS_INFO("radar mapping init");
}

void RadarMapping::cb_loop(const visualization_msgs::Marker loop)
{
    update_map();
}

void RadarMapping::cb_config(radar_odom::mappingConfig &config, uint32_t level)
{
    ROS_INFO("map snr thresh: %f", config.snr_thresh);
    ROS_INFO("map use voxel: %s", config.map_voxel ? "true" : "false");
    if (config.map_voxel)
        ROS_INFO("full map voxel: %f", config.voxel_size);

    use_voxel = config.map_voxel;
    snr_filter.setFilterLimits(config.snr_thresh, FLT_MAX);
    map_voxel_filter.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);

    if (full_path.poses.size() != 0)
        update_map();
}

void RadarMapping::update_map()
{
    uint step = full_path.poses.size();
    PointCloud<PointXYZSD>::Ptr map(new PointCloud<PointXYZSD>);
    for (uint i = 0; i < step; i++)
    {
        uint seq = full_path.poses[i].header.seq;
        auto it = cache.find(seq);
        if (it != cache.end())
        {
            PointCloud<PointXYZSD>::Ptr map_part(new PointCloud<PointXYZSD>);
            Pose3 p = toPose3(full_path.poses[i]);
            transformPointCloud(*it->second, *map_part, p.matrix());
            *map += *map_part;
        }
    }
    snr_filter.setInputCloud(map);
    snr_filter.filter(*map);
    if (use_voxel)
    {
        map_voxel_filter.setInputCloud(map);
        map_voxel_filter.filter(*map);
    }

    *full_map = *map;
    sensor_msgs::PointCloud2 map_msg;
    toROSMsg(*map, map_msg);
    map_msg.header.frame_id = "map";
    pub_map.publish(map_msg);
    map->clear();
}

void RadarMapping::cb_node(const sensor_msgs::PointCloud2ConstPtr radar_msg,
                           const nav_msgs::PathConstPtr path)
{

    PointCloud<PointXYZSD>::Ptr radar_pc(new PointCloud<PointXYZSD>);
    fromROSMsg(*radar_msg, *radar_pc);
    voxel_filter.setInputCloud(radar_pc);
    voxel_filter.filter(*radar_pc);
    for (size_t i = 0; i < radar_pc->size(); i++)
    { // log dB
        radar_pc->points[i].snr = log(radar_pc->points[i].snr);
    }

    cache[radar_msg->header.seq] = radar_pc;
    full_path = *path;

    PointCloud<PointXYZSD>::Ptr map_part(new pcl::PointCloud<pcl::PointXYZSD>);
    Pose3 p = toPose3(full_path.poses.back());

    transformPointCloud(*radar_pc, *map_part, p.matrix());
    snr_filter.setInputCloud(map_part);
    snr_filter.filter(*map_part);
    *full_map += *map_part;
    if (use_voxel)
    {
        map_voxel_filter.setInputCloud(full_map);
        map_voxel_filter.filter(*full_map);
    }

    sensor_msgs::PointCloud2 map_msg;
    toROSMsg(*full_map, map_msg);
    map_msg.header.frame_id = "map";
    pub_map.publish(map_msg);
    map_part->clear();
}

RadarMapping::~RadarMapping()
{
    delete sync;
}

int main(int argc, char *argv[])
{
    init(argc, argv, "radar_mapping");
    NodeHandle nh;
    RadarMapping rmap(&nh);
    spin();
    return 0;
}
