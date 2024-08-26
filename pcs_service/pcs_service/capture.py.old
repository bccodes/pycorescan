from pcs_interfaces.srv import Capture
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

import sys
from launch import LaunchService
from launch import LaunchDescription
from launch_ros.actions import Node as lNode


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Capture, 'capture', self.capture_callback)

        self.get_logger().info('Ready to capture')

    def capture_callback(self, request, response):
        self.get_logger().info('Received Capture Request')

        # self.launch_savers('hello')

        self.req = Empty.Request()

        self.left_client = self.create_client(Empty, '/left_saver/save')
        self.get_logger().warning('1')
        self.future = self.left_client.call_async(self.req)
        self.get_logger().warning('2')

        # while rclpy.ok():
        #     rclpy.spin_once(self)
        #     if self.future.done():
        #         #Get response
        #         break
        self.get_logger().warning('here')
        # rclpy.spin_until_future_complete(self, self.future)
        # self.client = MinimalClientAsync()
        # response.result = self.client.send_request()
        # self.client.destroy_node()
        return response

    def launch_savers(self, label):
        desc = self.generate_launch_description(label)
        service = LaunchService(argv=sys.argv[1:])
        service.include_launch_description(desc)
        return service.run()

    def generate_launch_description(self, label):
        return LaunchDescription([
            lNode(
                package="image_view",
                executable="image_saver",
                parameters=[{'save_all_image': False,
                             'filename_format': '/home/ben/' + label + '-left%04i.jpg'}],
                remappings=[("__ns", "/left_saver"),
                            ("image", "/lefty_raw")]
                ),

            lNode(
                package="image_view",
                executable="image_saver",
                parameters=[{'save_all_image': False,
                             'filename_format': '/home/ben/' + label + '-right%04i.jpg'}],
                remappings=[("__ns", "/right_saver"),
                            ("image", "/righty_raw")]
                )])


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')

        self.get_logger().info('started async client!!!!!!')
        
        self.left_client = self.create_client(Empty, '/left_saver/save')
        while not self.left_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for left saver...')
        else:
            self.get_logger().info('left saver OK')

        self.right_client = self.create_client(Empty, '/right_saver/save')
        while not self.left_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for right saver...')
        else:
            self.get_logger().info('right saver OK')

        self.req = Empty.Request()

    def send_request(self):
        self.future = self.left_client.call_async(self.req)
        self.get_logger().info('calling left saver')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('got left result!')

        self.future = self.right_client.call_async(self.req)
        self.get_logger().info('calling right saver')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('got right result!')
        return True


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
