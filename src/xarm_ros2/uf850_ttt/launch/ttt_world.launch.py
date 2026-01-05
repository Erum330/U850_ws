from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('uf850_ttt'),
        'worlds',
        'ttt_master.world'
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r --render-engine ogre ', world_file]
        }.items()
    )

    # ✅ Add ROS<->GZ bridge for the service so your python can call it directly
    # This replaces the manual:
    # ros2 run ros_gz_bridge parameter_bridge /world/ttt_world/set_pose@ros_gz_interfaces/srv/SetEntityPose
    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/world/ttt_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ]
    )

    return LaunchDescription([
        gz_sim,
        set_pose_bridge,
    ])
