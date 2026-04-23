"""
rviz.launch.py — Launch RViz2 with the mecanum_bot configuration.
Run this in a second terminal AFTER sim.launch.py is up.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mecanum_bot')
    rviz_config = os.path.join(pkg, 'config', 'mecanum_bot.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([rviz2])
