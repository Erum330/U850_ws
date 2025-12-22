import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Get the directory of your custom package
    package_name = 'uf850_ttt'
    pkg_share = get_package_share_directory(package_name)

    # 2. Path to your custom world file (already installed in share)
    world_path = os.path.join(pkg_share, 'worlds', 'ttt_master.world')

    # 3. Path to the official xarm_moveit_config launch file
    # This official file handles robot spawning, MoveIt2, and Gazebo controllers
    xarm_moveit_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('xarm_moveit_config'), 
            'launch', 'uf850_moveit_gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_path,
            'add_vacuum_gripper': 'true', # CRITICAL: Enables picking logic
            'robot_type': 'uf850',
        }.items(),
    )

    return LaunchDescription([
        xarm_moveit_gazebo_launch
    ])