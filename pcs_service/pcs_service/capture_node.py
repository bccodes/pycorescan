# This node currently does everything:
# Scheduling, Image grabbing, Saving, Relay control and GPIO control.
# 
# Subscribes to: /capture (CaptureRequest)
# Action Client to: /camera_name/grab_images_raw (GrabImages)
# Publishes to: /status (Bool)

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, String
from pcs_interfaces.msg import CaptureRequest, UpdateSettings
from pylon_ros2_camera_interfaces.action import GrabImages

from cv_bridge import CvBridge, CvBridgeError
import cv2

import serial
import os


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.label_prefix = ""
        self.exposure1 = 300000
        self.exposure2 = 200000

        self.job_subscription = self.create_subscription(
            CaptureRequest,
            '/capture',
            self.job_in_callback,
            10)
        self.settings_sub = self.create_subscription(
            UpdateSettings,
            '/update_settings',
            self.update_settings_callback,
            10)
        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback) # period in seconds 
        self.tasks_todo = []
        self.busy = False
        self.status_publisher = self.create_publisher(
            Bool,
            '/status',
            10)

        # self.has_serial = self.trySerialConnect()
        self.has_serial = True

        self.usb_path = self.get_usb_storage_path()
        if self.usb_path:
            self.has_storage = True
        else:
            self.has_storage = False
        
        self.status_msg = ""


    def timer_callback(self):
        """Every timer cycle, check if busy and if not, start a task.
        Then publish status."""
        ready_state = Bool() # status is true if ready to capture
        ready_state.data = False
        
        if not self.has_serial: # check for numato device 
            self.status_msg = "check relay box connection"
            self.has_serial = self.try_serial_connect()
        elif not self.has_storage: # check for usb storage device
            self.status_msg = "check usb storage connection"
            self.usb_path = self.get_usb_storage_path()
            if self.usb_path:
                self.has_storage = True
        elif self.tasks_todo and not self.busy: # start a new task
            # get a capture task and run it
            self.busy = True
            next_task = self.tasks_todo.pop(0)
            self.get_logger().info(f'running task: {next_task}')
            self.run_task(next_task)
        elif not self.tasks_todo: # ready
            if os.access(self.usb_path, os.W_OK):
                ready_state.data = True
                self.get_logger().info(f'ready to capture', throttle_duration_sec=10)
                self.status_msg = ""
            else: # we lost the usb?
                self.has_storage = False
        else: # busy
            if not self.active_job:
                self.get_logger().info(f'something went wrong... no active job but busy?', throttle_duration_sec=1)
            self.get_logger().info(f'working on job {self.active_job}', throttle_duration_sec=1)

        # publish status
        self.status_publisher.publish(ready_state)

        if ready_state.data and self.status_msg:
            self.get_logger().info(self.status_msg, throttle_duration_sec=3)
        elif self.status_msg:
            self.get_logger().warn(self.status_msg, throttle_duration_sec=3)


    def try_serial_connect(self):
        # open serial port for relay control
        try:
            self.serPort = serial.Serial('/dev/ttyACM0', 19200, timeout=3)
            # zero all relays
            for i in range(8):
                relay_num = str(i + 1)
                self.get_logger().info('zeroing relay ' + relay_num)
                cmd = 'relay off ' + relay_num + '\n\r'
                self.serPort.write(cmd.encode())
            success = True
        except:
            # self.get_logger().error('could not access /dev/ttyACM0, is numato board connected?', throttle_duration_sec=5)
            success = False
        return success

    
    def get_usb_storage_path(self):
        """check if any usb storage device is found and writable
        return the path of the usb, or an empty string if not found"""

        # self.get_logger().info('searching for usb')
        # Check if the /media/ directory exists
        media_path = '/media/'
        if not os.path.exists(media_path):
            return ""
        # Iterate through all directories in /media/
        for device in os.listdir(media_path):
            device_path = os.path.join(media_path, device)
        # Check for any subdirectory (usually the label) within the mounted device
            subdirs = [os.path.join(device_path, subdir) for subdir in os.listdir(device_path)]
            # Look for writable subdirectory
            for subdir in subdirs:
                if os.path.isdir(subdir) and os.access(subdir, os.W_OK):
                    # self.get_logger().info(f'found usb storage at {subdir}')
                    self.create_folder(subdir)
                    return subdir
        return ""


    def create_folder(self, path, name="corescans"):
        folder_path = os.path.join(path, name)
        try:
            # with open(folder_path, 'w') as f:
            #     pass  # This will create an empty file
            os.makedirs(name, exist_ok=True)
            self.get_logger().info(f"Working directory: {folder_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to create folder at {folder_path}: {e}")
            self.has_storage = False


    def job_in_callback(self, msg):
        """Every time a message hits /capture, check if busy, and if not,
        append the necessary tasks to tasks_todo."""
        if not self.tasks_todo:
            self.active_job = (msg.label1 + '-' + msg.label2)
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'ring'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'left',
                                    'label': self.active_job + '-ring',
                                    'exposure': msg.exposure1})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'right',
                                    'label': self.active_job + '-ring',
                                    'exposure': msg.exposure1})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'uv'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'left',
                                    'label': self.active_job + '-uv',
                                    'exposure': msg.exposure2})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'right',
                                    'label': self.active_job + '-uv',
                                    'exposure': msg.exposure2})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'ring'})
        
        self.get_logger().info(str(self.tasks_todo))


    def update_settings_callback(self, msg):
        if self.busy:
            self.get_logger().warn('cannot change settings while busy. try again')
        else:
            self.label_prefix = msg.label
            self.exposure1 = msg.exposure1
            self.exposure2 = msg.exposure2
            self.get_logger().info('successfully updated settings')
            self.get_logger().info(f'label: {msg.label}, e1: {msg.exposure1}, e2: {msg.exposure2}')

    def run_task(self, task):
        """Something about this just seems wrong and I'm sorry"""
        match task['type']:
            case 'lights':
                match task['mode']:
                    case 'ring':
                        self.switch_relay(1, True)
                        self.switch_relay(2, False)
                    case 'uv':
                        self.switch_relay(1, False)
                        self.switch_relay(2, True)
                    case _:
                        self.get_logger().warn('something went wrong bad task type')
                self.busy = False
            case 'capture':
                self.active_label = task['label']
                self.active_exposure = task['exposure']
                self.active_side = task['side']
                self.send_capture_goal()
                # Stay busy at this point so the scheduler will not accept tasks
                # until the current awaited image is saved.
            case _:
                self.get_logger().warn('something went wrong bad task type')
                self.busy = False

    def send_capture_goal(self):
        # start client for pylon wrapper
        match self.active_side:
            case 'left':
                self._cam_client = ActionClient(
                    self,
                    GrabImages,
                    '/my_camera/some_node/grab_images_raw')
                self._cam_client.wait_for_server()
            case 'right':
                self._cam_client = ActionClient(
                    self,
                    GrabImages,
                    '/my_camera/charlie_c/grab_images_raw')
                self._cam_client.wait_for_server()
            case _:
                self.get_logger().warn('something went wrong no active_side')
                self.busy = False


        # send capture request
        try:
            goal_msg = GrabImages.Goal()
            goal_msg.exposure_given = True
            goal_msg.exposure_times = [self.active_exposure]
            self._send_goal_future = self._cam_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        except AssertionError:
            self.get_logger().error(f'Bad config, type error')


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
            cv2.imwrite('/home/ben/nu_ws/' + self.active_label + '-' + self.active_side + '.png', cv2_img)
        except CvBridgeError as e:
            self.get_logger().error('CVBridgeError')
            print(e)
        self.busy=False

    def switch_relay(self, relay_num, state):
        cmd = 'relay ' + ('on ' if state else 'off ') + str(relay_num) 
        self.get_logger().info('sending relay command: ' + cmd)
        
        try:
            self.serPort.write(cmd.encode())
        except:
            self.has_serial = False

def main(args=None):
    rclpy.init(args=args)

    capture_node = CaptureNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(capture_node, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
