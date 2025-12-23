#!/usr/bin/env python3

import os
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file

def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    robot_type = 'uf850'
    dof = 6

    # 1. Custom World
    gazebo_world = PathJoinSubstitution([FindPackageShare('uf850_ttt'), 'worlds', 'ttt_master.world'])

    # 2. Robot Description
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'uf850_controllers.yaml'),
        prefix=prefix.perform(context), 
        add_gripper=False,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type
    )

    robot_description = {
        'robot_description': get_xacro_content(
            context,
            xacro_file=Path(get_package_share_directory('xarm_description')) / 'urdf' / 'xarm_device.urdf.xacro', 
            dof=dof,
            robot_type=robot_type,
            ros2_control_plugin='gz_ros2_control/GazeboSimSystem',
            ros2_control_params=ros2_control_params,
            add_gripper=True,
        )
    }

    # 3. Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': ['-r ', gazebo_world]}.items(),
    )

    # NEW SPAWN COORDINATES: Center-back of table
    gazebo_spawn_entity_node = Node(
        package="ros_gz_sim",
        executable="create",
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'UF_ROBOT',
            '-x', '0.0',    # Robot at back
            '-y', '0.0',    # Centered
            '-z', '0.751',  # Resting on table
            '-Y', '0.0',    # Facing forward
        ],
        parameters=[{'use_sim_time': True}],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 4. Controllers
    controllers = ['joint_state_broadcaster', 'uf850_traj_controller']
    controller_nodes = [Node(
        package='controller_manager', executable='spawner',
        arguments=[c], parameters=[{'use_sim_time': True}]
    ) for c in controllers]

    return [
        robot_state_publisher_node, gazebo_launch, gz_bridge,
        RegisterEventHandler(OnProcessStart(target_action=robot_state_publisher_node, on_start=gazebo_spawn_entity_node)),
        RegisterEventHandler(OnProcessExit(target_action=gazebo_spawn_entity_node, on_exit=controller_nodes)),
    ]

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])