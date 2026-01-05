#!/usr/bin/env python3
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart

from uf_ros_lib.uf_robot_utils import get_xacro_content, generate_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='ufactory')
    robot_type = 'uf850'
    dof = 6

    # ----------------------------
    # 1) World
    # ----------------------------
    gazebo_world = PathJoinSubstitution([
        FindPackageShare('uf850_ttt'),
        'worlds',
        'ttt_master.world'
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r --render-engine ogre ', gazebo_world]}.items(),
    )

    # ----------------------------
    # 2) MoveIt config
    # ----------------------------
    moveit_config_builder = MoveItConfigsBuilder(
        context=context,
        prefix=prefix,
        hw_ns=hw_ns,
        robot_type=robot_type,
        dof=dof,
        add_gripper=True
    )
    moveit_config_dict = moveit_config_builder.to_dict()

    # ----------------------------
    # 3) robot_description + ros2_control params
    # ----------------------------
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'uf850_controllers.yaml'),
        prefix=prefix.perform(context),
        add_gripper=True,
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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    gazebo_spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'UF_ROBOT',
            '-x', '0.0', '-y', '0.0', '-z', '0.751', '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------
    # 4) Bridges
    # ----------------------------
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/world/ttt_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
    )

    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/world/ttt_world/create@ros_gz_interfaces/srv/SpawnEntity'],
    )

    # ----------------------------
    # 4.1) Camera bridges (Gazebo -> ROS)
    # ----------------------------
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/ttt_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/ttt_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
    )

    # ----------------------------
    # 4.2) Spawn overhead camera model
    # ----------------------------
    camera_sdf = r"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="ttt_camera">
    <static>true</static>
    <link name="link">
      <sensor name="ttt_camera_sensor" type="camera">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>/ttt_camera/image</topic>
        <camera_info_topic>/ttt_camera/camera_info</camera_info_topic>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>1280</width>
            <height>720</height>
            <format>RGB_INT8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10.0</far>
          </clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""

    camera_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ttt_camera',
            '-string', camera_sdf,
            '-x', '0.4', '-y', '0.0', '-z', '1.35',
            '-R', '0.0', '-P', '1.57', '-Y', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ----------------------------
    # 5) MoveIt + RViz
    # ----------------------------
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config_dict, {'use_sim_time': True}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'rviz', 'moveit.rviz'])
        ],
        parameters=[moveit_config_dict, {'use_sim_time': True}],
    )

    # ----------------------------
    # 6) Controllers
    # ----------------------------
    controllers = [
        'joint_state_broadcaster',
        'uf850_traj_controller',
        'uf850_gripper_traj_controller'
    ]
    controller_nodes = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[c],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        for c in controllers
    ]

    # ----------------------------
    # 7) Pieces spawner
    # ----------------------------
    pieces_spawner_node = Node(
        package='uf850_ttt',
        executable='spawn_ttt_pieces',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return [
        gazebo_launch,

        # bridges early
        clock_bridge,
        set_pose_bridge,
        spawn_bridge,

        # camera bridge
        camera_bridge,

        # publish robot model
        robot_state_publisher_node,

        # moveit + rviz
        move_group_node,
        rviz_node,

        # spawn robot after rsp starts
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[gazebo_spawn_entity_node]
            )
        ),

        # spawn camera shortly after Gazebo starts
        TimerAction(period=2.0, actions=[camera_spawn_node]),

        # controllers + pieces after robot spawned
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo_spawn_entity_node,
                on_exit=controller_nodes + [
                    TimerAction(period=2.0, actions=[pieces_spawner_node])
                ]
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
