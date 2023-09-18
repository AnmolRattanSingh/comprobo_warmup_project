import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Teleop(Node):

    def __init__(self):
        super().__init__('teleop_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key
    
    def run_loop(self):
        while self.key != '\x03':
            key = self.getKey()
            
            # move forward if 'w' is pressed
            if key == 'w':
                self.drive(0.5, 0.0)
                print("driving forward")
            # move backward if 's' is pressed
            elif key == 's':
                self.drive(-0.5, 0.0)
                print("driving backward")
            # turn left if 'a' is pressed
            elif key == 'a':
                self.drive(0.0, 1.0)
                print("turning left")
            # turn right if 'd' is pressed
            elif key == 'd':
                self.drive(0.0, -1.0)
                print("turning right")
            # stop if 'x' is pressed
            elif key == 'e':
                self.drive(0.0, 0.0)
                print("stopping")

    def drive(self, linear_vel, angular_vel):
        """ Send a drive command to the robot """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)       # Initialize communication with ROS
    node = Teleop()             # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    node.destroy_node()         # Cleanup resources
    rclpy.shutdown()            # Cleanup


if __name__ == '__main__':
    main()
