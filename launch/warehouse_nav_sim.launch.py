"""
Full simulation stack for the T500 warehouse demo, in one launch file.

Combines, in order, what used to be four manual terminals:
  1. ros2 launch t500_description warehouse.launch.py
  2. ros2 launch nav2_bringup slam_launch.py use_sim_time:=True
  3. ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
  4. rviz2

Gazebo/SLAM/Nav2/RViz are staggered with TimerAction so that each stage
only starts once the previous one has had time to come up (TF tree,
/scan, lifecycle services). Starting nav2_bringup's SLAM or navigation
launch before Gazebo has spawned the robot and published /scan is the
usual cause of "service not available, waiting..." timeouts, so the
delays below default generously and are all overridable from the CLI.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    t500_share = get_package_share_directory('t500_description')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(t500_share, 'params', 'nav2_smac.yaml')
    default_rviz_config = os.path.join(t500_share, 'config', 'display.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    slam_delay = LaunchConfiguration('slam_delay')
    nav_delay = LaunchConfiguration('nav_delay')
    rviz_delay = LaunchConfiguration('rviz_delay')

    # 1) Gazebo + robot_state_publisher + joint_state_publisher + spawn t500
    warehouse = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(t500_share, 'launch', 'warehouse.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # 2) SLAM (slam_toolbox), delayed so Gazebo/the robot are up first
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )
    slam_timer = TimerAction(period=slam_delay, actions=[slam])

    # 3) Nav2 navigation stack, delayed until after SLAM is publishing /map
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items(),
    )
    navigation_timer = TimerAction(period=nav_delay, actions=[navigation])

    # 4) RViz, started last once TF/map/costmaps exist to visualize
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz_timer = TimerAction(period=rviz_delay, actions=[rviz])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2/SLAM params yaml'
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='Full path to the RViz config file'
        ),
        DeclareLaunchArgument(
            'slam_delay', default_value='10.0',
            description='Seconds to wait after Gazebo before starting SLAM'
        ),
        DeclareLaunchArgument(
            'nav_delay', default_value='16.0',
            description='Seconds to wait after Gazebo before starting Nav2 navigation'
        ),
        DeclareLaunchArgument(
            'rviz_delay', default_value='19.0',
            description='Seconds to wait after Gazebo before starting RViz'
        ),

        warehouse,
        slam_timer,
        navigation_timer,
        rviz_timer,
    ])
