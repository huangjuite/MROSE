import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    info_plot_arg = DeclareLaunchArgument(
        "info_plot", default_value="false", description="Enable info plot"
    )
    config_arg = DeclareLaunchArgument(
        name="config",
        default_value="config_epson",
        description="config_epson, config_razor...",
    )

    def odom_node_setup(context):
        doppler_odom_node = Node(
            package="rpl_rio",
            executable="runDIO",
            output="screen",
            parameters=[
                {
                    "config": os.path.join(
                        get_package_share_directory("rpl_rio"),
                        "cfg",
                        f"{LaunchConfiguration('config').perform(context)}.yaml",
                    )
                }
            ],
        )
        return [doppler_odom_node]

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=[
            "-l",
            PathJoinSubstitution([FindPackageShare("rpl_rio"), "cfg", "info_vis.xml"]),
        ],
        condition=IfCondition(LaunchConfiguration("info_plot")),
    )

    ld = LaunchDescription([info_plot_arg, config_arg, plotjuggler_node])
    ld.add_action(OpaqueFunction(function=odom_node_setup))
    return ld
