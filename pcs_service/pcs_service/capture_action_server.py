import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from pcs_interfaces.msg import CaptureRequest
from pylon_ros2_camera_interfaces.action import GrabImages


class CaptureNode(Node):

    def __init__(self):
        super().__init__('capture_node')
        # listen on /jobs_in
        self.job_subscription = self.create_subscription(
           CaptureRequest,
           'jobs_in',
           run_capture_job,
           10)
           

    def run_capture_job(self, msg):
        job = (msg.label1, msg.label2)
        self.grab_images_client = ActionClient(self, GrabImages, 'grab_client')
        self.grab_images_client.wait_for_server()
        self.get_logger().info(f'Running capture job {job}')
        self.send_goal(msg)

    def send_goal(self, capture_request):
        goal_msg = GrabImages.Goal()
        goal_msg.brightness_given = True
        goal_msg.brightness_values = [1]
        self._send_goal_future = self.grab_images_client.send_goal_async(goal_msg)
