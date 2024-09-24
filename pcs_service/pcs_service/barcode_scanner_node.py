import rclpy
from rclpy.node import Node
from pcs_interfaces.msg import CaptureRequest

import evdev


class BarcodeScannerNode(Node):
    def __init__(self):
        super().__init__('barcode_scanner_node')
        self.publisher_ = self.create_publisher(
                CaptureRequest,
                '/capture',
                10)
        self.get_logger().info('Barcode scanner node up')

        self.device_path = '/dev/input/by-id/usb-OPTO-E_Barcode_Device-event-kbd'
        
        self.create_timer(0.2, self.timer_callback)
        self.has_scanner = False

        self.barcode_data = ''

    def timer_callback(self):
        if not self.has_scanner:
            try:
                self.device = evdev.InputDevice(self.device_path)
                self.get_logger().info(f'Connected to device {self.device.name}')
                self.listen_to_scanner()
            except Exception as e:
                self.get_logger().warning(f'Could not connect to device: {e}', throttle_duration_sec=3)
                return


    def listen_to_scanner(self):
        for event in self.device.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                key_event = evdev.categorize(event)
                if key_event.keystate == evdev.KeyEvent.key_down:
                    keycode = key_event.keycode

                    if keycode == 'KEY_ENTER':
                        msg = CaptureRequest()
                        msg.segment_id = self.barcode_data.strip()
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'published barcode: {msg.segment_id}')
                        self.barcode_data = ''
                    else:
                        char = evdev.ecodes.KEY[key_event.scancode]
                        self.barcode_data += char.replace("KEY_", "").lower()


    def destroy_node(self):
        self.device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeScannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down barcode scanner node')
    finally:
        node.destroy_node()
        rclpy.shutdown()
