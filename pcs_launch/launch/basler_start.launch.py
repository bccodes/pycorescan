from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    this_package_name = 'pcs_launch'

    basler_ld1 = IncludeLaunchDescription(
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

    basler_ld2 = IncludeLaunchDescription(
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

    capture_node = Node(
            package='pcs_service',
            executable='capture_node',
            name='capture_node'
            )

    barcode_scanner_node = Node(
            package='pcs_service',
            executable='barcode_scanner_node',
            name='barcode_scanner_node'
            )

    led_switcher_node = Node(
            package='pcs_service',
            executable='led_switcher_node',
            name='led_switcher_node'
            )

    # bridge = Node(
    #         package="foxglove_bridge",
    #         executable="foxglove_bridge",
    #         name="foxglove_bridge"
    #         )
    bridge = Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket"
            )
    
    return LaunchDescription([
        basler_ld1,
        basler_ld2,
        capture_node,
        barcode_scanner_node,
        led_switcher_node,
        bridge,
        ])
