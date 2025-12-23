import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Force the exact path to your world
    # This ensures Gazebo doesn't load the default 'beside_table' world
    world_path = os.path.expanduser('~/U850_ws/src/xarm_ros2/uf850_ttt/worlds/ttt_master.world')

    # 2. Use the 'uf850_moveit_gazebo' launch file
    # This is the master file that handles MoveIt2 + Gazebo + Robot Spawning
    robot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_config'),
            'launch',
            'uf850_moveit_gazebo.launch.py'
        ])),
        launch_arguments={
            'world': world_path,
            'add_vacuum_gripper': 'true',
            'robot_type': 'uf850',
            'gz_type': 'gz',
        }.items(),
    )

    return LaunchDescription([
        robot_simulation_launch
    ])