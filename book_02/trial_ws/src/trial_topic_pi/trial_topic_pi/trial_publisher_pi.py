import rclpy
import numpy as np
from rclpy.node import Node
from trial_interface.msg import Valu

class TrialPublisher(Node):
    def __init__(self):
        super().__init__("trial_publisher")
        self.publisher_ = self.create_publisher(Valu, "trial_topic", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1
    def timer_callback(self):
        msg = Valu()
        msg.valu = self.i * np.sin(np.radians(360 / self.i)) / 2
        self.publisher_.publish(msg)
        self.get_logger().info("publishing, '%s'" % msg.valu)
        self.i += 1

def main(args=None):
    try:
        rclpy.init(args=args)
        trial_publisher = TrialPublisher()
        rclpy.spin(trial_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        trial_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


