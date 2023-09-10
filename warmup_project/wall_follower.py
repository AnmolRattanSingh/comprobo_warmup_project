import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from time import sleep
import math


class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.scan = None
        self.odom = None
        self.Flag = True

    def odom_callback(self, msg):
        self.odom = msg
    
    def scan_callback(self, msg):
        self.scan = msg
    
    def run_loop(self):
        # Check if scan messages are available
        if self.scan is not None:
            # Get the distances to the left and right walls
            front_distance = self.scan.ranges[0]
            left_distance = self.scan.ranges[45]
            right_distance = self.scan.ranges[135]
            
            print("front distance: ", front_distance)
            print("left distance: ", left_distance)
            print("right distance: ", right_distance)
            # Check if the robot is too close to the front wall
            if front_distance < 1:
                # Turn away from the wall
                self.turn_left_deg(-1.0, 20)
            # Check if the distances are equal
            elif abs(left_distance - right_distance) < 0.5:
                # Stop the robot
                self.drive(0.5, 0.0)
                sleep(0.2)
            elif left_distance < right_distance:
                # Turn clockwise
                self.turn_left_deg(-1.0, 2)
            else:
                # Turn anticlockwise
                self.turn_left_deg(1.0, 2)
    
    def run_loop2(self):
        # Check if scan messages are available
        if self.scan is not None:
            # Get the distances to the left and right walls
            fd = self.scan.ranges[0]
            d1 = self.scan.ranges[45]
            d2 = self.scan.ranges[135]
            bd = self.scan.ranges[180]
            
            theta = math.acos((d1+d2)/math.sqrt(2*(d1**2+d2**2)))
            # turn amount theta
            # turn left
            # theta into degrees
            theta = (theta * 180) / math.pi
            
            if bd < 1:
                self.drive_forward(0.5, 1)
            elif fd < 1:
                self.turn_left_deg(-1.0, 90)
                sleep(2)
            elif theta > 5:
                print("theta: ", theta)
                self.turn_left_deg(1.0, theta)
                sleep(2)
            else:
                self.drive_forward(0.5, 1)
                sleep(2)
    
    def turn_left_deg(self, angular_vel, degrees):
        ACC_CONST = 0.0
        """ Turn left using drive """
        self.drive(0.0, angular_vel)
        turn_radians = (degrees * math.pi) / 180.0
        sleep(abs(turn_radians / angular_vel) + ACC_CONST)
        self.drive(0.0, 0.0)
        if angular_vel < 0:
            print(f"turned right {degrees} degrees")
        else:
            print(f"turned left {degrees} degrees")
    
    def drive_forward(self, linear_vel, distance):
        """ Drive forward """
        self.drive(linear_vel, 0.0)
        sleep(distance / linear_vel)
        self.drive(0.0, 0.0)
    
    def drive(self, linear_vel, angular_vel):
        """ Send a drive command to the robot """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)       # Initialize communication with ROS
    node = WallFollowNode()             # Create our Node
    rclpy.spin(node)            # Run the Node until ready to shutdown 
    node.destroy_node()         # Cleanup resources
    rclpy.shutdown()            # Cleanup


if __name__ == '__main__':
    main()
