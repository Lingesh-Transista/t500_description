"""
Real-robot bring-up + AMCL localization + Nav2 navigation for the T500.

Brings up, on actual hardware (no Gazebo):
  - robot_state_publisher (URDF -> TF)
  - joint_state_publisher (wheel joint TF, static placeholder -- real encoder
    angles are not published, only base pose matters for navigation)
  - diff_tf (encoder-fused odometry: odom -> base_footprint, from /lwheel,
    /rwheel, /cmd_vel)
  - YDLidar driver (-> /scan)
  - Nav2 AMCL localization against a saved map
  - Nav2 navigation stack (planner/controller/behaviors)

See the package README / chat history for the full hardware checklist
(motor driver -> /cmd_vel, encoder publisher -> /lwheel /rwheel, lidar
wiring, calibrated TICKS_PER_REV, etc.) before relying on this for
unattended navigation.
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('t500_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_share, 'map', 'shopfloor.yaml')
    default_params = os.path.join(pkg_share, 'params', 'nav2_smac.yaml')
    default_xacro = os.path.join(pkg_share, 'urdf', 't500.xacro')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_port = LaunchConfiguration('lidar_port')

    robot_description_config = xacro.process_file(default_xacro)
    robot_description = robot_description_config.toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time},
        ],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    diff_tf = Node(
        package='t500_description',
        executable='diff_tf',
        name='diff_tf_precise',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # NOTE: verify these against your exact YDLidar model (baud rate, sample
    # rate, intensity support, angle range) -- defaults here are placeholders.
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[{
            'port': lidar_port,
            'frame_id': 'lidar',
            'baudrate': 128000,
            'lidar_type': 1,
            'device_type': 0,
            'sample_rate': 9,
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 16.0,
            'range_min': 0.1,
            'frequency': 10.0,
            'reversion': False,
            'inverted': True,
            'auto_reconnect': True,
            'isSingleChannel': False,
            'intensity': False,
        }],
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Full path to the saved map yaml for AMCL localization'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Must be false on real hardware'
        ),
        DeclareLaunchArgument(
            'lidar_port', default_value='/dev/ydlidar',
            description='Serial port for the YDLidar (set up a udev rule for a stable name)'
        ),

        robot_state_publisher,
        joint_state_publisher,
        diff_tf,
        ydlidar_node,
        localization,
        navigation,
    ])
