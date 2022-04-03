import rclpy
from turtlesim.srv import Spawn


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("spawn_client")
    client = node.create_client(Spawn, "/spawn")
    req = Spawn.Request()
    req.x = 2.0
    req.y = 2.0
    req.theta = 0.2
    req.name = "new_turtle"

    while not client.wait_for_service(timeout_sec=10):
        node.get_logger().info("service not available, waiting again...")
    
    future = client.call_async(req)
    rclpy.spin_util_future_complete(node, future)
    try:
        result = future.result()
    except Exception as e:
        node.get_logger().info("Service call failed %r" % (e,))
    else:
        node.get_logger().info(
            "Result of x, y, theta, name: %f %f %f %s" % (req.x, req.y, req.theta, result.name)
        )
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
