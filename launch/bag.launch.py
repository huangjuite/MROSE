import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_arg = DeclareLaunchArgument(
        name="bag",
        default_value="/home/rpldocker/data/nrec/jeep/processed/soergel_0",
        description="Path to the data directory",
    )

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="none",
        description="rerun log mode (spawn, connect, save, server, none, etc.)",
    )

    config_arg = DeclareLaunchArgument(
        name="config",
        default_value="config_jeep",
        description="Config file name (without .yaml extension)",
    )

    traj_save_arg = DeclareLaunchArgument(
        name="traj_save",
        default_value="/tmp/trajectory.txt",
        description="Path to save trajectory output file",
    )

    vel_save_arg = DeclareLaunchArgument(
        name="vel_save",
        default_value="/tmp/velocity.txt",
        description="Path to save velocity output file",
    )

    def bagDIO_node_setup(context):
        bagDIO_node = Node(
            package="rpl_rio",
            executable="bagDIO",
            output="screen",
            emulate_tty=True,
            arguments=[
                LaunchConfiguration("bag"),
                LaunchConfiguration("mode"),
                LaunchConfiguration("traj_save"),
                LaunchConfiguration("vel_save"),
            ],
            parameters=[
                {
                    "config": os.path.join(
                        get_package_share_directory("rpl_rio"),
                        "cfg",
                        f"{LaunchConfiguration('config').perform(context)}.yaml",
                    ),
                }
            ],
        )
        return [bagDIO_node]

    ld = LaunchDescription([
        bag_arg,
        mode_arg,
        config_arg,
        traj_save_arg,
        vel_save_arg,
    ])
    ld.add_action(OpaqueFunction(function=bagDIO_node_setup))
    return ld
