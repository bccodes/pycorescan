import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = "pycorescan"

    cam_config_1 = os.path.join(
      get_package_share_directory('pcs_ros'),
      'config',
      'config1.yaml'
    )

    cam_config_2 = os.path.join(
      get_package_share_directory('pcs_ros'),
      'config',
      'config2.yaml'
    )

    saver_config = os.path.join(
      get_package_share_directory('pcs_ros'),
      'config',
      'saver.yaml'
    )
    return LaunchDescription([
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge"
        ),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            # arguments=["-p", cam_config_1],
            parameters=[cam_config_1],
            remappings=[("__ns", "/usb_cam_0")],
            name="cam_0"
        ),
        
        # Node(
        #     package="usb_cam",
        #     executable="usb_cam_node_exe",
        #     arguments=["-p", cam_config_2],
        #     remappings=[("__ns", "/usb_cam_1")],
        #     name="cam_1"
        # ),
        #
        Node(
            package="image_view",
            executable="image_saver",
            parameters=[
                {'save_all_image': False}
            ],
            remappings=[("image", "/usb_cam_0/image_raw")]

        )
    ])
