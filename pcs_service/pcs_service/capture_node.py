# This node currently does everything:
# Scheduling, Image grabbing, Saving, Relay control and (todo) GPIO control.
# 
# Subscribes to: /capture (CaptureRequest)
# Action Client to: /camera_name/grab_images_raw (GrabImages)
# Publishes to: /status (Bool)

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, String, Float32
from pcs_interfaces.msg import CaptureRequest, UpdateSettings, SwitchLights
from pylon_ros2_camera_interfaces.action import GrabImages

from cv_bridge import CvBridge, CvBridgeError
import cv2

import os
import time

SIMULATE_RELAYS = False
SIMULATE_CAMERAS = False
HIGHEST_EXPOSURE = 450000
LOWEST_EXPOSURE = 100

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.label_prefix = 'nolabel'
        self.declare_parameter('prefix', 'noprefix')
        self.declare_parameter('exp_ring', 300000.0)
        self.declare_parameter('exp_uv', 200000.0)

        self.exposure_ring = 300000.0
        self.exposure_uv = 200000.0

        self.trigger_sub = self.create_subscription(
            CaptureRequest,
            'capture',
            self.job_in_callback,
            10)
        # self.settings_sub = self.create_subscription(
        #     UpdateSettings,
        #     '/update_settings',
        #     self.update_settings_callback,
        #     10)
        self.prefix_sub = self.create_subscription(
                String,
                '/set_prefix',
                self.set_prefix_callback,
                10)
        self.exp_ring_sub = self.create_subscription(
                Float32,
                '/set_exposure_ring',
                self.set_exp_ring_callback,
                10)
        self.exp_uv_sub = self.create_subscription(
                Float32,
                '/set_exposure_uv',
                self.set_exp_uv_callback,
                10)


        self.status_publisher = self.create_publisher(
            Bool,
            'ready_status',
            10)
        self.storage_publisher = self.create_publisher(
            Bool,
            'has_storage',
            10)
        self.switch_lights_publisher = self.create_publisher(
            SwitchLights,
            '/switch_lights',
            10)

        self.cv_bridge = CvBridge()
        self.timer = self.create_timer(0.2, self.timer_callback) # period
        self.tasks_todo = []
        self.busy = False

        self.usb_path = self.get_usb_storage_path()
        if self.usb_path:
            self.has_storage = True
        else:
            self.has_storage = False
        
        self.status_msg = ""


    def set_prefix_callback(self, msg):
        if self.busy:
            self.get_logger().warn('cannot change settings while busy')
            return
        self.label_prefix = msg.data
        # my_new_param = rclpy.parameter.Parameter(
        #         'prefix',
        #         rclpy.Parameter.Type.STRING,
        #         msg.data
        #         )
        # self.set_parameters([my_new_param])
        self.get_logger().info(f'set prefix: {msg.data}')

    def set_exp_ring_callback(self, msg):
        if self.busy:
            self.get_logger().warn('cannot change settings while busy')
            return
        try:
            data = float(msg.data)
            assert (data <= HIGHEST_EXPOSURE)
            assert (data >= LOWEST_EXPOSURE)
        except:
            self.get_logger().error(f'invalid exposure for ring: {msg.data}')
            return
        self.exposure_ring = msg.data
        # my_new_param = rclpy.parameter.Parameter(
        #         'exp_ring',
        #         rclpy.Parameter.Type.DOUBLE,
        #         msg.data
        #         )
        # self.set_parameters([my_new_param])
        self.get_logger().info(f'applied ring exposure: {msg.data}')

    def set_exp_uv_callback(self, msg):
        if self.busy:
            self.get_logger().warn('cannot change settings while busy')
            return
        try:
            data = float(msg.data)
            assert (data <= HIGHEST_EXPOSURE)
            assert (data >= LOWEST_EXPOSURE)
        except:
            self.get_logger().error(f'invalid exposure for uv: {msg.data}')
            return
        self.exposure_uv = msg.data
        # my_new_param = rclpy.parameter.Parameter(
        #         'exp_uv',
        #         rclpy.Parameter.Type.DOUBLE,
        #         msg.data
        #         )
        # self.set_parameters([my_new_param])
        self.get_logger().info(f'applied uv exposure: {msg.data}')


    def timer_callback(self):
        """Every timer cycle, check if busy and if not, start a task.
        Then publish status."""
        ready_state = Bool() # status is true if ready to capture
        ready_state.data = False

        usb_state = Bool()
        
        
        if not self.has_storage: # check for usb storage device
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
                # self.get_logger().info(f'ready to capture', throttle_duration_sec=10)
                self.status_msg = ""
            else: # we lost the usb?
                self.has_storage = False
        else: # busy
            self.get_logger().info(f'number of tasks remaining: {len(self.tasks_todo)}')
            if not self.active_job:
                self.get_logger().info(f'something went wrong... no active job but busy?', throttle_duration_sec=1)
            self.get_logger().info(f'working on job {self.active_job}', throttle_duration_sec=1)

        # publish status
        usb_state.data = self.has_storage
        self.storage_publisher.publish(usb_state)
        self.status_publisher.publish(ready_state)

        # if ready_state.data and self.status_msg:
        #     self.get_logger().info(self.status_msg, throttle_duration_sec=3)
        # elif self.status_msg:
        #     self.get_logger().warn(self.status_msg, throttle_duration_sec=3)

    
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
                    self.get_logger().info(f'found storage at {subdir}')
                    return subdir
        # self.get_logger().warn(f'no usb storage detected', throttle_duration_sec=5)
        return ""


    def create_folder(self, path, name="corescans"):
        date_string = time.strftime('%d_%m_%Y')
        self.folder_path = os.path.join(path, name, date_string)
        try:
            if not os.path.isdir(self.folder_path):
                os.makedirs(self.folder_path, exist_ok=True)
            self.get_logger().info(f"Working directory: {self.folder_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to create folder at {self.folder_path}: {e}")
            self.has_storage = False


    # def touch_file(self, label):
    #     self.file_path = os.path.join(self.folder_path, label)
    #     try:
    #         with open(self.file_path, 'w') as f:
    #             pass  # create an empty file
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to touch file at {self.file_path}: {e}")
    #         self.has_storage = False


    # def update_settings_callback(self, msg):
    #     if self.busy:
    #         self.get_logger().warn('cannot change settings while busy')
    #         return
    #     if msg.core_id:
    #         self.label_prefix = msg.core_id
    #     else:
    #         self.label_prefix = "nolabel"
    #
    #     # validate exposures
    #     for exp in (msg.exposure_ring, msg.exposure_uv):
    #         if exp != '':
    #             try:
    #                 if exp > HIGHEST_EXPOSURE or exp < LOWEST_EXPOSURE:
    #                     self.get_logger().error(f'invalid exposure: {exp}')
    #                     return
    #             except:
    #                 self.get_logger().error(f'invalid exposure: {exp}')
    #                 return
    #
    #     if msg.exposure_ring:
    #         self.exposure_ring = msg.exposure_ring
    #     if msg.exposure_uv:
    #         self.exposure_uv = msg.exposure_uv
    #     self.get_logger().info('successfully updated settings')
    #     # self.get_logger().info(f'label: {msg.core_id}, e1: {msg.exposure_ring}, e2: {msg.exposure_uv}')


    def job_in_callback(self, msg):
        """Every time a message hits /capture, check if busy, and if not,
        append the necessary tasks to tasks_todo."""
        if not self.tasks_todo:
            if not msg.segment_id:
                label_suffix = 'noid'
            else:
                label_suffix = msg.segment_id

            label_prefix = self.get_parameter('prefix').value
            # exp_ring = self.get_parameter('exp_ring').value
            # exp_uv = self.get_parameter('exp_uv').value
            exp_ring = self.exposure_ring
            exp_uv = self.exposure_uv
            
            self.active_job = (label_prefix + '-' + label_suffix)
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'ring',
                                    'side': 'left'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'left',
                                    'label': self.active_job + '-ring',
                                    'exposure': exp_ring})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'ring',
                                    'side': 'right'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'right',
                                    'label': self.active_job + '-ring',
                                    'exposure': exp_ring})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'uv',
                                    'side': 'left'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'left',
                                    'label': self.active_job + '-uv',
                                    'exposure': exp_uv})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'uv',
                                    'side': 'right'})
            self.tasks_todo.append({'type': 'capture',
                                    'side': 'right',
                                    'label': self.active_job + '-uv',
                                    'exposure': exp_uv})
            self.tasks_todo.append({'type': 'lights',
                                    'mode': 'preview',
                                    'side': ''})
        
        # self.get_logger().info(str('\n'.join(self.tasks_todo)))
        self.get_logger().info(f'number of tasks remaining: {len(self.tasks_todo)}')


    def run_task(self, task):
        """Get a task which may either switch lights or request capture"""
        match task['type']:
            case 'lights':
                msg = SwitchLights()
                msg.mode = task['mode']
                msg.side = task['side']
                self.switch_lights_publisher.publish(msg) 
                self.busy = False

            case 'capture':
                self.active_label = task['label']
                self.active_exposure = task['exposure']
                self.active_side = task['side']
                self.time_string = time.strftime('%H-%M-%S')
                if not SIMULATE_CAMERAS:
                    self.send_capture_goal()
                    # Stay busy at this point so the scheduler will not accept
                    # tasks until the current awaited image is saved.
                else:
                    file_label = str(f'{self.active_label}-{self.active_side}-{self.time_string}')
                    self.touch_file(file_label)
                    self.get_logger().info(f'simulated capture: {file_label}.png')
                    self.busy = False

            case _:
                self.get_logger().warn('something went wrong bad task type')
                self.busy = False


    def send_capture_goal(self):
        # make an action client for the pylon camera 
        match self.active_side:
            case 'left':
                self._cam_client = ActionClient(
                    self,
                    GrabImages,
                    '/my_camera/camera_left/grab_images_raw')
                self._cam_client.wait_for_server()
            case 'right':
                self._cam_client = ActionClient(
                    self,
                    GrabImages,
                    '/my_camera/camera_right/grab_images_raw')
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
            cv2_img = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite(self.folder_path + '/' + self.time_string + '-' + self.active_label + '-' + self.active_side + '-' + str(int(self.active_exposure)) + '.png', cv2_img)
        except CvBridgeError as e:
            self.get_logger().error('CVBridgeError')
            print(e)
        self.busy=False


def main(args=None):
    rclpy.init(args=args)

    capture_node = CaptureNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(capture_node, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
