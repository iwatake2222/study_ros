import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTB3(Node):
    def __init__(self):
        super().__init__("tb3_move")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tmr = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = -0.1
        self.pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    move = MoveTB3()
    rclpy.spin(move)

if __name__ == "__main__":
    main()
