from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    this_package_name = "pcs_launch"
    www_directory = "/home/ben/nu_ws/src/pycorescan/www/"

    basler_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("pylon_ros2_camera_wrapper"),
                        "launch",
                        "pylon_ros2_camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare(this_package_name), "config", "basler1.yaml"]
            ),
            "node_name": "camera_left",
            "extra_arguments": "--ros-args --log-level WARN",
        }.items(),
    )

    basler_2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("pylon_ros2_camera_wrapper"),
                        "launch",
                        "pylon_ros2_camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution(
                [FindPackageShare(this_package_name), "config", "basler2.yaml"]
            ),
            "node_name": "camera_right",
        }.items(),
    )

    capture_node = Node(
        package="pcs_service", executable="capture_node", name="capture_node"
    )

    barcode_scanner_node = Node(
        package="pcs_service",
        executable="barcode_scanner_node",
        name="barcode_scanner_node",
    )

    led_switcher_node = Node(
        package="pcs_service", executable="led_switcher_node", name="led_switcher_node"
    )

    ros_bridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
    )

    http_server = ExecuteProcess(
        cmd=["python3", "-m", "http.server", "8000", "--directory", www_directory],
        output="screen",
    )

    return LaunchDescription(
        [
            basler_1_launch,
            basler_2_launch,
            capture_node,
            barcode_scanner_node,
            led_switcher_node,
            ros_bridge,
            http_server,
        ]
    )
