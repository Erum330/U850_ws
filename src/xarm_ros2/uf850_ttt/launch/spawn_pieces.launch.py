from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Bridge for the spawn service (CRITICAL)
    spawn_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/ttt_world/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )
    
    # Node to spawn Tic-Tac-Toe pieces
    spawn_node = Node(
        package='uf850_ttt',
        executable='spawn_ttt_pieces',
        output='screen'
    )
    
    return LaunchDescription([
        spawn_bridge,
        TimerAction(period=2.0, actions=[spawn_node])
    ])