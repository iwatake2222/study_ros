import rclpy
from rclpy.node import Node

from trial_interface.srv import XYZ

class TrialService(Node):
    def __init__(self):
        super().__init__("trial_service")
        self.srv = self.create_service(XYZ, "trial_service", self.square_sum_callback)

    def square_sum_callback(self, request, response):
        response.sqsum = request.x ** 2 + request.y ** 2 + request.z ** 2
        self.get_logger().info("Incoming request\nx: %d, y: %d, z: %d, sqsum: %d" % (request.x, request.y, request.z, response.sqsum))
        return response

def main(args=None):
    try:
        rclpy.init(args=args)
        trial_server = TrialService()
        rclpy.spin(trial_server)
    except KeyboardInterrupt:
        pass
    finally:
        trial_server.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
