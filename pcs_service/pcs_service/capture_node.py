import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from pcs_interfaces.msg import CaptureRequest
from pylon_ros2_camera_interfaces.action import GrabImages

from cv_bridge import CvBridge, CvBridgeError
import cv2


class CaptureNode(Node):

    def __init__(self):
        super().__init__('capture_node')
        self.job_subscription = self.create_subscription(
            CaptureRequest,
            '/capture',
            self.job_in_callback,
            10)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback) # period in seconds 
        self.captures_todo = []
        self.busy = False
        self.status_publisher = self.create_publisher(
            Bool,
            '/status',
            10)


    def timer_callback(self):
        status_msg = Bool() # status is true if ready to capture
        status_msg.data = False
        if self.captures_todo and not self.busy:
            self.busy = True
            self.status_publisher.publish(status_msg)
            # get a capture task and run it
            next_capture = self.captures_todo.pop(0)
            self.get_logger().info(f'running task: {next_capture}')
            self.run_task(next_capture)
        elif not self.captures_todo:
            status_msg.data = True
            self.status_publisher.publish(status_msg)
            self.get_logger().info(f'ready to capture', throttle_duration_sec=3)

    def job_in_callback(self, msg):
        if not self.captures_todo:
            job_name = (msg.label1 + '-' + msg.label2)
            self.captures_todo.append({'label': job_name + '-ring',
                                       'exposure': msg.exposure1})
            self.captures_todo.append({'label': job_name + '-uv',
                                       'exposure': msg.exposure2})


    def run_task(self, task):
        self.active_label = task['label']
        self.active_exposure = task['exposure']
        self.right_cam_client = ActionClient(
            self,
            GrabImages,
            '/my_camera/charlie_c/grab_images_raw')
        self.right_cam_client.wait_for_server()
        try:
            self.send_goal(self.active_exposure)
        except AssertionError:
            self.get_logger().error(f'Bad config, type error')


    def send_goal(self, exposure):
        goal_msg = GrabImages.Goal()
        goal_msg.exposure_given = True
        goal_msg.exposure_times = [exposure]
        self._send_goal_future = self.right_cam_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.busy = False
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        if (len(result.images) > 0):
            self.save_image(result.images[0])


    def save_image(self, image):
        self.get_logger().info('Saving image ' + self.active_label)
        try:
            cv2_img = self.cv_bridge.imgmsg_to_cv2(image, "rgb8")
            cv2.imwrite('/home/ben/nu_ws/' + self.active_label +'.png', cv2_img)
        except CvBridgeError as e:
            self.get_logger().error('CVBridgeError')
            print(e)
        self.busy=False


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
