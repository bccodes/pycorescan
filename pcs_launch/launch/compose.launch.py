from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    this_package_name = 'pcs_launch'

    ld1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('pylon_ros2_camera_wrapper'),
                    'launch',
                    'pylon_ros2_camera.launch.py'
                    ])
                ]),
            launch_arguments={
                'config_file': PathJoinSubstitution([
                    FindPackageShare(this_package_name),
                    'config',
                    'basler1.yaml'
                    ]),
                'node_name': 'some_node'
                }.items()
            )

    ld2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('pylon_ros2_camera_wrapper'),
                    'launch',
                    'pylon_ros2_camera.launch.py'
                    ])
                ]),
            launch_arguments={
                'config_file': PathJoinSubstitution([
                    FindPackageShare(this_package_name),
                    'config',
                    'basler2.yaml'
                    ]),
                'node_name': 'charlie_c'
                }.items()
            )

    bridge = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge"
            )


    return LaunchDescription([
        ld1,
        ld2,
        bridge
        ])
