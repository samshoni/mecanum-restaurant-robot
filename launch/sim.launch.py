"""
sim.launch.py — Mecanum Bot full simulation launch

Odometry strategy (same as working tortoisebot):
  /scan → RF2O Laser Odometry → /odom_laser → EKF → /odom + odom→base_footprint TF
  This avoids depending on the Gazebo odometry bridge (unreliable lazy activation).

Topics bridged:
  /clock      Ign→ROS  (immediate — use_sim_time nodes need this first)
  /cmd_vel    ROS→Ign  (delayed 6 s — same bridge timing as scan)
  /scan       Ign→ROS  (delayed 6 s — RF2O and SLAM subscribe before bridge starts)
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('mecanum_bot')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg, 'worlds', 'restaurant_world.sdf')
    model_sdf  = os.path.join(pkg, 'models', 'mecanum_bot', 'model.sdf')
    urdf_file  = os.path.join(pkg, 'urdf',   'mecanum_bot.urdf.xacro')
    ekf_params = os.path.join(pkg, 'config', 'ekf.yaml')

    models_dir = os.path.join(pkg, 'models')
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=models_dir + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
    )

    robot_description = xacro.process_file(urdf_file).toxml()

    # ── 1. Ignition Gazebo ────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── 2. Spawn robot (3 s delay so Gazebo is ready) ────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'mecanum_bot',
                    '-file', model_sdf,
                    '-x', '0.0', '-y', '0.0', '-z', '0.05',
                    '-R', '0.0', '-P', '0.0', '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # ── 3. Robot State Publisher ──────────────────────────────────────────
    #    use_sim_time=False: fixed joints publish to /tf_static at wall t=0,
    #    which is valid for all time regardless of sim clock.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # ── 4a. Clock bridge (immediate) ──────────────────────────────────────
    #    Starts first so RF2O, EKF, and SLAM can receive /clock and register
    #    their /scan subscriptions before the scan bridge starts.
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_clock',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # ── 4b. Scan + cmd_vel bridge (delayed 6 s) ───────────────────────────
    #    By t=6 s, RF2O (/scan subscriber) and SLAM (/scan subscriber) are
    #    already running → bridge finds subscribers immediately on startup
    #    → subscribes to Ignition /scan right away.
    bridge_sensors = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge_sensors',
                output='screen',
                arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                    '/model/mecanum_bot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                ],
            )
        ],
    )

    # ── 5. RF2O Laser Odometry ────────────────────────────────────────────
    #    Same as tortoisebot: reads /scan, publishes /odom_laser (no TF).
    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_laser',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': True,
        }],
    )

    # ── 6. EKF — publishes /odom and odom→base_footprint TF ──────────────
    #    Same as tortoisebot: fuses /odom_laser, publishes filtered /odom
    #    and broadcasts odom→base_footprint TF.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
        remappings=[('odometry/filtered', '/odom')],
    )

    return LaunchDescription([
        ign_resource_path,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge_clock,
        bridge_sensors,
        rf2o,
        ekf,
    ])
