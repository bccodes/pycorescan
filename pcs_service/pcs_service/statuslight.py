import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from pcs_interfaces.msg import CaptureRequest

class StatusLight(Node):

    def __init__(self):
        super().__init__('statuslight')
        # create status bool publisher, true = ready to accept capture request
        self.status_publisher = self.create_publisher(
            Int32,
            '/status',
            10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_status)
        # create a set for storing currently awaited jobs
        self.jobs_todo = set()
        # make a subscriber to /capture
        self.capture_subscription = self.create_subscription(
            CaptureRequest,
            '/capture',
            self.capture_request_callback,
            10)
        self.capture_subscription
        # make a publisher for forwarding requests to capture action node
        self.jobs_in_publisher = self.create_publisher(
            CaptureRequest,
            '/jobs_in',
            10)
        self.jobs_done_subscription = self.create_subscription(
            CaptureRequest,
            '/jobs_done',
            self.job_done_callback,
            10)

    def publish_status(self):
        # publish ready true if there are no jobs todo
        msg = Int32()
        has_jobs = (len(self.jobs_todo) != 0) # its either true or false
        if has_jobs:
            msg.data = 1
            self.get_logger().info(f'Has jobs: {self.jobs_todo}',
            throttle_duration_sec=3)
        else: # no jobs duh
            msg.data = 0
        self.status_publisher.publish(msg)
        self.get_logger().info(f'No Jobs, status {msg.data}',
            throttle_duration_sec=3)

    def capture_request_callback(self, msg):
        # first check that we are not working on a job, and if so return early
        has_jobs = (len(self.jobs_todo) != 0) # its either true or false
        if has_jobs:
            self.get_logger().info(f'Rejected Request, too busy: {msg.label1} {msg.label2}')
            return

        # wait for capture requests and add them to the jobs todo list
        # also publish the new job to /jobs_in
        # msg will be of type 'CaptureRequest', {label1:String, label2:String}
        self.get_logger().info(f'New capture request: {msg.label1} {msg.label2}')
        # add the job to the local set of todos (as a tuple)
        self.jobs_todo.add((msg.label1, msg.label2))
        # forward the message to /jobs_in
        self.jobs_in_publisher.publish(msg)

    def job_done_callback(self, msg):
        # wait for messages on /jobs_done then remove them from the local todos
        # this should free up the status to ready true after all jobs are done
        try:
            self.jobs_todo.remove((msg.label1, msg.label2))
            self.get_logger().info(f'Done job: {msg.label1} {msg.label2}')
        except KeyError:
            self.get_logger().error(f'Unknown done job {msg.label1} {msg.label2}')


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
