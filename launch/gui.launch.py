"""Launch the restaurant operator GUI.

Run AFTER sim.launch.py + nav2.launch.py are up:
  ros2 launch mecanum_bot gui.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mecanum_bot",
            executable="restaurant_gui.py",
            name="restaurant_gui",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
