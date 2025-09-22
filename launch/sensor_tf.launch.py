#!/usr/bin/env python3

import numpy as np
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_razor',
            arguments=[
                '0.0', '0.0', '0.0',    # x, y, z translation
                '0.0', '0.0', '0.0',    # yaw, pitch, roll rotation (radians)
                'base_link',            # parent frame
                'razor'                 # child frame
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar',
            arguments=[
                '0.05', '0.0', '0.3',       # x, y, z translation
                f'{np.pi/2}', '0.0', f'{np.pi}',   # yaw, pitch, roll rotation (radians)
                'base_link',                # parent frame
                'radar'                     # child frame
            ]
        ),
        
    ])