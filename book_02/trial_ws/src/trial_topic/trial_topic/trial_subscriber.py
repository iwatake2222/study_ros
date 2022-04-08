from typing import final
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrialSubscriber(Node):
    def __init__(self):
        super().__init__("trial_subscriber")
        self.subscription = self.create_subscription(
            String,
            "trial_topic",
            self.listener_callback,
            10)
        self.subscription
    def listener_callback(self, msg):
        self.get_logger().info("Subscribed, '%s'" % msg.data)

def main(args=None):
    try:
        rclpy.init(args=args)
        trial_subscriber = TrialSubscriber()
        rclpy.spin(trial_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        trial_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
