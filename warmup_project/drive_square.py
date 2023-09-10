""" This script explores publishing ROS messages in ROS using Python """
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class DrawSquareNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.square_done = False

    def run_loop(self):
        if not self.square_done:
            for _ in range(4):
                print("driving forward")
                self.drive_forward(0.5, distance=1)
                print("turning left")
                self.turn_left_deg(1.0, degrees=90.0)
        self.square_done = True

    def drive(self, linear_vel, angular_vel):
        """ Send a drive command to the robot """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)
    
    def turn_left_deg(self, angular_vel, degrees):
        """ Turn left using drive """
        self.drive(0.0, angular_vel)
        turn_radians = (degrees * math.pi) / 180.0
        sleep(turn_radians / angular_vel + 0.09)
        self.drive(0.0, 0.0)
    
    def drive_forward(self, linear_vel, distance):
        """ Drive forward """
        self.drive(linear_vel, 0.0)
        sleep(distance / linear_vel)
        self.drive(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = DrawSquareNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    node.destroy_node()        # Cleanup resources
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
