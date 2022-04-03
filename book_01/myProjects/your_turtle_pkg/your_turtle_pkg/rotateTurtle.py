import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlesim.action import RotateAbsolute

class RotateTurtle(Node):
    def __init__(self):
        super().__init__("rotate_turtle")

        self._action_client = ActionClient(
            self, RotateAbsolute, "/turtle1/rotate_absolute"
        )
    
    def send_goal(self, theta):
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = RotateTurtle()
    action_client.send_goal(0.0)

if __name__ == "__main__":
    main()
