import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pcs_interfaces.msg import CaptureRequest

import sys
import select

import tty
import termios


class BarcodeScannerNode(Node):
    def __init__(self):
        super().__init__('barcode_scanner_node')
        self.publisher_ = self.create_publisher(
                CaptureRequest,
                '/capture',
                10)
        self.get_logger().info('Barcode scanner node up bebe')

        # store terminal settings
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # self.create_timer(0.1, self.scan_barcode)
        self.barcode_data = ''


    def scan_barcode(self):
        while rclpy.ok():
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                char = sys.stdin.read(1)
                # print(char)
                if char == '\r' or char == '\n':
                    msg = CaptureRequest()
                    msg.segment_id = self.barcode_data.strip()
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'published barcode: {msg.segment_id}')
                    self.barcode_data = ''
                else:
                    self.barcode_data += char

    
    def destroy_node(self):
        # restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeScannerNode()

    try:
        # rclpy.spin(node)
        node.scan_barcode()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down barcode scanner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()
