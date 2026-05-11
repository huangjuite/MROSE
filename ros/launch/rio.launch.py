import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        name="config",
        description="config_epson, config_razor...",
    )
    use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    def odom_node_setup(context):
        doppler_odom_node = Node(
            package="mrose",
            executable="runDIO",
            output="screen",
            parameters=[
                {
                    "config": os.path.join(
                        get_package_share_directory("mrose"),
                        "cfg",
                        f"{LaunchConfiguration('config').perform(context)}.yaml",
                    ),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }
            ],
        )
        
        return [doppler_odom_node]

    ld = LaunchDescription([config_arg, use_sim_time])
    ld.add_action(OpaqueFunction(function=odom_node_setup))
    return ld
