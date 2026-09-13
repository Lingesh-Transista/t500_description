from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Package paths
    pkg_share = get_package_share_directory('t500_description')
    world_path = os.path.join(pkg_share, 'worlds', 'shopfloor.world')
    xacro_path = os.path.join(pkg_share, 'urdf', 't500.xacro')

    # Process xacro
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = robot_description_config.toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Publishes /joint_states for the wheel joints so robot_state_publisher
    # can complete the left_wheel/right_wheel transforms (Gazebo's
    # publish_wheel_tf does not emit them on this plugin version).
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 't500',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot,
    ])
