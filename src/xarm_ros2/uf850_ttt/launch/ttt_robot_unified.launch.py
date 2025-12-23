import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Path to your custom Tic-Tac-Toe world
    world_file = PathJoinSubstitution([
        FindPackageShare('uf850_ttt'),
        'worlds',
        'ttt_master.world'
    ])

    # 2. Start Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': ['-r --render-engine ogre ', world_file]}.items()
    )

    # 3. Get the Robot Description (URDF) 
    # This uses the official xarm_description package to generate the model
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xarm_description'),
                'launch',
                'uf850_rviz_display.launch.py'
            ])
        ]),
        launch_arguments={'add_vacuum_gripper': 'true'}.items()
    )

    # 4. The Spawner Node
    # This node takes the robot model and places it ON the table (z=0.75)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'UF_ROBOT',
            '-topic', 'robot_description',
            '-x', '0.2',   # Adjust these to center the robot on your table
            '-y', '0.0',
            '-z', '0.75'   # Table height
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        robot_description_launch,
        spawn_robot
    ])