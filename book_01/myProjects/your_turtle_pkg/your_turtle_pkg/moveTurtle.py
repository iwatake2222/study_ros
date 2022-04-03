import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveTurtle(Node):
    def __init__(self):
        super().__init__("turtlesim_move")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.tmr = self.create_timer(1.0, self.timer_callback)
        self.sub = self.create_subscription(Pose, "turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg):
        self.get_logger().info("(x, y, theta):[%f %f %f]" % (msg.x, msg.y, msg.theta))

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    move = MoveTurtle()
    rclpy.spin(move)

if __name__ == "__main__":
    main()
