from tkinter import E
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class TrialParam(Node):
    def __init__(self):
        super().__init__("trial_parameter_publisher")
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.declare_parameter("parameter")

    def timer_callback(self):
        initial_param = self.get_parameter("parameter").get_parameter_value().string_value
        self.get_logger().info("Initial param is %s" % initial_param)
        new_param = rclpy.parameter.Parameter(
            "parameter",
            rclpy.Parameter.Type.STRING,
            "Tokyo"
        )
        new_parameters = [new_param]
        self.set_parameters(new_parameters)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = TrialParam()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
