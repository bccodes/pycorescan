# A node to own the serial link to the numato usb relay box
# 
# Should receive a message with light side (left or right) and type (ring or uv)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from pcs_interfaces.msg import SwitchLights

import serial

SIMULATE_RELAYS = False

class LightSwitcherNode(Node):
    def __init__(self):
        super().__init__('light_switcher_node')
        
        if SIMULATE_RELAYS:
            self.has_serial = True
        else:
            self.has_serial = self.try_serial_connect()
        
        self.task_sub = self.create_subscription(
                SwitchLights,
                '/switch_lights',
                self.switch_lights_callback,
                10)

        self.status_publisher = self.create_publisher(
                Bool,
                'has_relays',
                10)

        self.status_timer = self.create_timer(
                1,
                self.timer_callback)
                

    def timer_callback(self):
        self.has_serial = self.try_serial_connect()
        status_msg = Bool()
        status_msg.data = self.has_serial
        selt.status_publisher.publish(status_msg)
    
    def try_serial_connect(self):
        try:
            self.serPort = serial.Serial('/dev/ttyACM0', 19200, timeout=3)
            self.get_logger().info('Numato Relay Box Connected.', throttle_duration_sec=5)
            success = True
        except Exception as e:
            self.get_logger().error('could not access numato board, check connection', throttle_duration_sec=5)
            # print(e)
            success = False
        return success


    def zero_all_relays(self):
        for i in range(8):
            relay_num = str(i + 1)
            self.get_logger().info('zeroing relay ' + relay_num)
            cmd = 'relay off ' + relay_num + '\n\r'
            self.serPort.write(cmd.encode())
    

    def switch_lights_callback(self, msg):
        self.get_logger().info('switching lights...')
        task = {'mode': msg.mode,
                'side': msg.side}
        print(task)

        match task['mode']:
            case 'ring':
                match task['side']:
                    case 'left':
                        self.switch_relay(1, True)
                        self.switch_relay(2, False)
                        self.switch_relay(3, False)
                        self.switch_relay(4, False)
                    case 'right':
                        self.switch_relay(1, False)
                        self.switch_relay(2, True)
                        self.switch_relay(3, False)
                        self.switch_relay(4, False)
            case 'uv':
                match task['side']:
                    case 'left':
                        self.switch_relay(1, False)
                        self.switch_relay(2, False)
                        self.switch_relay(3, True)
                        self.switch_relay(4, False)
                    case 'right':
                        self.switch_relay(1, False)
                        self.switch_relay(2, False)
                        self.switch_relay(3, False)
                        self.switch_relay(4, True)
            case 'preview': # both ring lights on
                self.switch_relay(1, True)
                self.switch_relay(2, True)
                self.switch_relay(3, False)
                self.switch_relay(4, False)
            case _:
                self.get_logger().warn('something went wrong bad task type')


    def switch_relay(self, relay_num, state):
        if not SIMULATE_RELAYS:
            cmd = 'relay ' + ('on ' if state else 'off ') + str(relay_num) 
            self.get_logger().info('sending relay command: ' + cmd)
            try:
                self.serPort.write(cmd.encode())
            except:
                self.has_serial = False


def main(args=None):
    rclpy.init(args=args)

    light_switcher_node = LightSwitcherNode()

    executor = MultiThreadedExecutor()

    rclpy.spin(light_switcher_node, executor)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
