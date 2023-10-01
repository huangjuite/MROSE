# Radar Inertial Odometry

## dependencies
- ROS noetic
- GTSAM 4.2a8
- Ceres-Solver 2.1.0


## dataset
[sheet_link](https://docs.google.com/spreadsheets/d/1Y8laH8Bpx8Q06hx8cPZvMHAJ0SMbmu_COp1SMDYQfNc/edit#gid=0)

## quick start

1. rviz
```
roslaunch radar_odom vis_mapping.launch 
```

2. radar slam
```
roslaunch radar_odom rpl_rio.launch
```

3. play bag
```
rosbag play RPL_radar/1669749443/rosbag/* --clock
```