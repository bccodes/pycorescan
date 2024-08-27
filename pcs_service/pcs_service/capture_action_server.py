import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


from pcs_interfaces.msg import CaptureRequest
from pylon_ros2_camera_interfaces.action import GrabImages

from cv_bridge import CvBridge, CvBridgeError
import cv2

class CaptureNode(Node):

    def __init__(self):
        super().__init__('capture_node')
        # listen on /jobs_in
        self.job_subscription = self.create_subscription(
            CaptureRequest,
            'jobs_in',
            self.run_capture_job,
            10)
        self.done_publisher = self.create_publisher(
            CaptureRequest,
            'jobs_done',
            10)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(5, clock=self.get_clock())

    def run_capture_job(self, msg):
        self.active_job = msg
        job = (msg.label1, msg.label2)
        self.get_logger().info(f'Got new job {job}')
        self.grab_images_client = ActionClient(
            self,
            GrabImages,
            '/my_camera/charlie_c/grab_images_raw')
        self.get_logger().info(f'Waiting for Pylon action server...')
        self.grab_images_client.wait_for_server()
        self.get_logger().info(f'Running capture job {job}')
        try:

            self.lightmode='ring'
            self.request_images(msg)
            self.grab_images_client.wait_for_server()
            # delay = self.create_timer(5)
            # rclpy.spin_until_future_complete(delay)
            self.get_logger().warning('here')
            self.timer.sleep()
            self.get_logger().warning('222')

            self.lightmode='uv'
            self.request_images(msg, is_uv=True)
            self.grab_images_client.wait_for_server()
        except AssertionError:
            self.get_logger().error(f'Bad config, type error')
            self.done_publisher.publish(self.active_job)

    def request_images(self, msg, is_uv=False):
        goal_msg = GrabImages.Goal()
        goal_msg.exposure_given = True
        if is_uv:
            goal_msg.exposure_times = [msg.exposure2] #uv led exposure
        else:
            goal_msg.exposure_times = [msg.exposure1] #ring light exposure

        self._send_goal_future = self.grab_images_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if (len(result.images) > 0):
            self.get_logger().info('Saving image..')
            label = self.active_job.label1 + '-' + self.active_job.label2
            self.save_image(label, result.images[0])

    def save_image(self, label, image):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.cv_bridge.imgmsg_to_cv2(image, "rgb8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a png 
            cv2.imwrite('/home/ben/nu_ws/' + label + '-' + self.lightmode +'.png', cv2_img)


def main(args=None):
    rclpy.init(args=args)

    capture_node = CaptureNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(capture_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    capture_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
