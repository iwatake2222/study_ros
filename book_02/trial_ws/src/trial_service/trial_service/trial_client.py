import sys
import rclpy
from rclpy.node import Node

from trial_interface.srv import XYZ

class TrialClientAsync(Node):
    def __init__(self):
        super().__init__("trial_client")
        self.cli = self.create_client(XYZ, "trial_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available")
        self.req = XYZ.Request()
    
    def send_request(self):
        self.req.x = int(sys.argv[1])
        self.req.y= int(sys.argv[2])
        self.req.z = int(sys.argv[3])
        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    trial_client = TrialClientAsync()
    trial_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(trial_client)
        if trial_client.future.done():
            try:
                response = trial_client.future.result()
            except Exception as e:
                trial_client.get_logger().info("Service call failed %r" % (e, ))
            else:
                trial_client.get_logger().info(
                    "Result of square sum: %d^2 + %d^2 %d^2 = %d" % (trial_client.req.x, trial_client.req.y, trial_client.req.z, response.sqsum)
                )
            break
    trial_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
