
#include <ros/ros.h>
#include "MapOptimize.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_pose_graph");
    ros::NodeHandle nh;
    MapOptimize<pcl::PointXYZSDW> rpg(&nh);

    std::thread map_thread(&MapOptimize<pcl::PointXYZSDW>::render_map, &rpg);

    ros::spin();
    return 0;
}