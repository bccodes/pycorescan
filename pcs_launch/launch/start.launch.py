import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    this_package_name = 'pcs_launch'

    cam_config_1 = os.path.join(
      get_package_share_directory(this_package_name),
      'config',
      'config1.yaml'
    )

    cam_config_2 = os.path.join(
      get_package_share_directory(this_package_name),
      'config',
      'config2.yaml'
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge"
    )

    left_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[cam_config_1],
        remappings=[("__ns", "/usb_cam_0"),
                    ("image_raw", "/lefty_raw")],
        name="cam_0"
    )
    
    right_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        parameters=[cam_config_2],
        remappings=[("__ns", "/usb_cam_1"),
                    ("image_raw", "/righty_raw")],
        name="cam_1"
    )

    left_saver = Node(
        package="image_view",
        executable="image_saver",
        parameters=[{'save_all_image': False,
                     'filename_format': 'left%04i.jpg'}],
        remappings=[("__ns", "/left_saver"),
                    ("image", "/lefty_raw")]
    )

    right_saver = Node(
        package="image_view",
        executable="image_saver",
        parameters=[{'save_all_image': False,
                     'filename_format': 'right%04i.jpg'}],
        remappings=[("__ns", "/right_saver"),
                    ("image", "/righty_raw")]
    )

    return LaunchDescription([
        left_cam,
        # right_cam,
        left_saver,
        # right_saver,
        foxglove_bridge
    ])
