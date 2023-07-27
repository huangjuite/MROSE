
#include <ros/ros.h>
#include "MapOptimize.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_pose_graph");
    ros::NodeHandle nh;
    MapOptimize<pcl::PointXYZISW> rpg(&nh);

    std::thread map_thread(&MapOptimize<pcl::PointXYZISW>::render_map, &rpg);

    ros::spin();
    return 0;
}