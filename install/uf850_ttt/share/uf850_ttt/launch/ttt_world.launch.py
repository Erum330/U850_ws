from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Path to your custom Tic-Tac-Toe world file
    # This uses FindPackageShare to dynamically find your uf850_ttt package
    world_file = PathJoinSubstitution([
        FindPackageShare('uf850_ttt'),
        'worlds',
        'ttt_master.world'
    ])

    # 2. Include the standard Gazebo Sim launch file
    # We pass 'gz_args' to set the render engine and the world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            # -r runs the simulation immediately on startup
            # --render-engine ogre is best for VMware/Linux environments
            'gz_args': ['-r --render-engine ogre ', world_file]
        }.items()
    )

    return LaunchDescription([
        gz_sim
    ])