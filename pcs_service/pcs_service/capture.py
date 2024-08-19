from pcs_interfaces.srv import Capture
from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node

    
class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Capture, 'capture', self.capture_callback)

        self.client = MinimalClientAsync()
        self.get_logger().info('Ready to capture')

    def capture_callback(self, request, response):
        self.get_logger().info('Received Capture Request')
        response.result = self.client.send_request()

        return response


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')

        self.left_client = self.create_client(Empty, '/left_saver/save')
        while not self.left_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for left saver...')
        else:
            self.get_logger().info('left saver OK')

        self.req = Empty.Request()

    def send_request(self):
        self.future = self.left_client.call_async(self.req)
        self.get_logger().info('calling left saver')
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info('got left result!')
        return True


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
