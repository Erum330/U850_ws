from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uf850_ttt',
            executable='pick_place_one',
            output='screen'
        )
    ])