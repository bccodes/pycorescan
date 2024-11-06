import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from pcs_interfaces.msg import SetParameterMsg  # Replace with actual package name

class ParameterClient(Node):
    def __init__(self):
        super().__init__('parameter_client_node')
        
        # Create service client for setting parameters
        self.cli = self.create_client(SetParameters, '/my_camera/camera_left/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for target node to become available...')
        
        # Create subscriber to listen for parameter setting requests on /set_parameter
        self.subscription = self.create_subscription(
            SetParameterMsg,
            '/set_parameter',
            self.parameter_callback,
            10
        )
        self.get_logger().info('Parameter client ready to receive parameter updates')

    def parameter_callback(self, msg):
        # Extract parameter details from the message
        param_name = msg.param_name
        param_value = msg.param_value
        param_type = msg.param_type
        
        # Set the parameter
        result = self.set_parameter(param_name, param_value, param_type)
        if result:
            self.get_logger().info(f'Successfully set parameter {param_name} to {param_value}')
        else:
            self.get_logger().error(f'Failed to set parameter {param_name}')
        return

    def set_parameter(self, param_name, param_value, param_type):
        # Create the Parameter object
        param = Parameter()
        param.name = param_name
        param_value_msg = ParameterValue()
        
        # Set the parameter type and value based on msg data
        if param_type == "String":
            param_value_msg.type = 4
            param_value_msg.string_value = param_value
        elif param_type == "Double":
            param_value_msg.type = 3
            param_value_msg.double_value = float(param_value)
        else:
            self.get_logger().error("Unsupported parameter type")
            return None
        
        param.value = param_value_msg
        request = SetParameters.Request()
        request.parameters = [param]
        
        self.get_logger().info('here')
        # Send the request and wait for the result
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        # Check if the future is done and return result
        if future.done():
            return future.result()
        else:
            self.get_logger().error('Service call timed out')
            return None

def main(args=None):
    rclpy.init(args=args)
    parameter_client = ParameterClient()
    rclpy.spin(parameter_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

