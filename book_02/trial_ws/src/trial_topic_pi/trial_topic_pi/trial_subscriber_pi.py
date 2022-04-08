import rclpy
from rclpy.node import Node
from trial_interface.msg import Valu

class TrialSubscriber(Node):
    def __init__(self):
        super().__init__("trial_subscriber")
        self.subscription = self.create_subscription(
            Valu,
            "trial_topic",
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info("subscribed, '%s'" % msg.valu)

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


