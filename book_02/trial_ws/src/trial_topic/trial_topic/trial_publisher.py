from typing import final
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrialPublisher(Node):
    def __init__(self):
        super().__init__("trial_publisher")
        self.publisher_ = self.create_publisher(String, "trial_topic", 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
    def timer_callback(self):
        msg = String()
        msg.data = "Hello! %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing, '%s'" % msg.data)
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
