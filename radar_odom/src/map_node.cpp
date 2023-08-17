
#include <ros/ros.h>
#include "MapOptimize.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "radar_pose_graph");
    ros::NodeHandle nh;
    MapOptimize<pcl::PointXYZSD> rpg(&nh);

    std::thread map_thread(&MapOptimize<pcl::PointXYZSD>::render_map, &rpg);

    ros::spin();
    return 0;
}