from pcs_interfaces.srv import Capture
from pylon_ros2_camera_interfaces.action import GrabImages

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Capture, 'capture', self.capture_callback)

        self.get_logger().info('Ready to capture')

    def capture_callback(self, request, response):
        self.get_logger().info('Received Capture Request')

        try:
            self.get_logger().info('Label 1: ' + request.label1)
            self.get_logger().info('Label 2: ' + request.label2)
        
            
            # put some code to call the cameras. it will return with a message,
            # which then needs to be saved as an image file.
            # steps:
            # set the ring light on

            # send capture request
            
            # store result as label1-label2-ring-left
            # repeat for left/right ring/uv for all 4 images
            

        except AttributeError:
            self.get_logger().info('Message Type Error... need both labels! -BW')
        response.result = True
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
