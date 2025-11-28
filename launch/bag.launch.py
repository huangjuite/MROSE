import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    data_dir_arg = DeclareLaunchArgument(
        name="data_dir",
        default_value="/home/rpldocker/data/nrec/jeep/processed/soergel_0",
        description="Path to the data directory",
    )

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="server",
        description="bagDIO mode (server, client, etc.)",
    )

    config_arg = DeclareLaunchArgument(
        name="config",
        default_value="config_jeep",
        description="Config file name (without .yaml extension)",
    )

    def bagDIO_node_setup(context):
        bagDIO_node = Node(
            package="rpl_rio",
            executable="bagDIO",
            output="screen",
            arguments=[LaunchConfiguration("data_dir"), LaunchConfiguration("mode")],
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

    ld = LaunchDescription([data_dir_arg, mode_arg, config_arg])
    ld.add_action(OpaqueFunction(function=bagDIO_node_setup))
    return ld
