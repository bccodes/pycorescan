import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

from pcs_interfaces.msg import CaptureRequest

class StatusLight(Node):

    def __init__(self):
        super().__init__('statuslight')
        # create status bool publisher, true = ready to accept capture request
        self.status_publisher = self.create_publisher(
            Bool,
            '/status',
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # create a set for storing currently awaited jobs
        self.jobs_todo = set()
        # make a subscriber to /capture
        self.capture_subscription = self.create_subscription(
            CaptureRequest,
            '/capture',
            self.listener_callback,
            10)
        self.capture_subscription
        # make a publisher for forwarding requests to capture action node
        self.jobs_in_publisher = self.create_publisher(
            CaptureRequest,
            '/jobs_in',
            10)

    def timer_callback(self):
        # publish true if there are no jobs todo
        msg = Bool()
        if (len(self.jobs_todo) == 0):
            msg.data = True
        else:
            msg.data = False
        self.status_publisher.publish(msg)
        self.get_logger().info(f'Publishing Ready: {msg.data}', throttle_duration_sec=3)

    def listener_callback(self, msg):
        # wait for capture requests and add them to the jobs todo list
        # also publish the new job to /jobs_in
        # msg will be of type 'CaptureRequest', {label1:String, label2:String}
        self.get_logger().info(f'Adding new job: {msg.label1} {msg.label2}')
        # add the job to the local set of todos
        self.jobs_todo.add((msg.label1, msg.label2))
        # forward the message to /jobs_in




def main(args=None):
    rclpy.init(args=args)

    statuslight_pub = StatusLight()

    rclpy.spin(statuslight_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    statuslight_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
