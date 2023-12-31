
""" This script explores publishing ROS messages in ROS using Python """
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

ACC_ERROR = 0.09  # Account for acceleration and deceleration

class DrawSquareNode(Node):
    def __init__(self):
        """
        Initialize the ROS node, sets up a timer for the run loop, 
        creates a publisher for velocity commands, and initializes 
        a flag to keep track of whether the square is completed.
        """
        super().__init__('send_message_node')
        timer_period = 0.1  # Create a timer that fires ten times per second
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.square_done = False

    def run_loop(self):
        """
        Executes the main loop that drives the robot in a square pattern. 
        Calls `drive_forward` and `turn_left_deg` in sequence.
        """
        if not self.square_done:
            for _ in range(4):
                print("driving forward")
                self.drive_forward(0.5, distance=1)
                print("turning left")
                self.turn_left_deg(1.0, degrees=90.0)
            self.square_done = True
        else:
            self.drive(0.0, 0.0)
            return

    def drive(self, linear_vel, angular_vel):
        """
        Publishes a Twist message to control the robot's linear and angular velocity.
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)
    
    def turn_left_deg(self, angular_vel, degrees):
        """
        Turns the robot left by a certain number of degrees at a given angular velocity.
        """
        self.drive(0.0, angular_vel)
        turn_radians = (degrees * math.pi) / 180.0
        sleep(turn_radians / angular_vel + ACC_ERROR)
        self.drive(0.0, 0.0)
    
    def drive_forward(self, linear_vel, distance):
        """
        Drives the robot forward for a certain distance at a given linear velocity.
        """
        self.drive(linear_vel, 0.0)
        sleep(distance / linear_vel)
        self.drive(0.0, 0.0)

def main(args=None):
    """
    Initializes ROS, creates the node, and spins it until shutdown.
    """
    rclpy.init(args=args)
    node = DrawSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
