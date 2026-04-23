"""
slam.launch.py — Online async SLAM with slam_toolbox

Run AFTER sim.launch.py is already up:
  ros2 launch mecanum_bot slam.launch.py

Publishes:  /map  (nav_msgs/OccupancyGrid)
TF chain:   map → odom → base_footprint → base_link → lidar_link
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mecanum_bot')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        slam_toolbox,
    ])
