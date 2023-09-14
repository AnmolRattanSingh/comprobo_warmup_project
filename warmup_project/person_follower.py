""" This script explores publishing ROS messages in ROS using Python """
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

DIST_THRESHOLD = 1.0

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('send_message_node')
        # Create a timer that fires ten times per second
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.person_angle = None
        self.person_dist = None
        self.scan = None
    
    def get_lidar_coords(self, msg):
        theta = 0
        self.scan = []
        for distance in msg.ranges:
            if distance !=0 and not math.isinf(distance):
                range_coords = (math.cos(theta) * distance, math.sin(theta) * distance)
                self.scan.append(range_coords)
            theta += msg.angle_increment
    
    def on_scan(self, msg):
        self.get_lidar_coords(msg)
        if self.scan:
            lidar_centroid = np.array(self.scan).mean(axis=0)
            self.person_dist = np.sqrt(np.sum(lidar_centroid ** 2))
            self.person_angle = math.atan2(lidar_centroid[1], lidar_centroid[0])

            if lidar_centroid[0] < 0:
                self.person_angle = -self.person_angle
                self.person_dist = -self.person_dist

            print(self.person_dist, self.person_angle)

    def run_loop(self):
        if self.person_dist is not None:
            # Person is too far away, use proportional control to drive towards them
            Kp_linear = 0.7  # Proportional gain for linear velocity
            Kp_angular = 1.0  # Proportional gain for angular velocity
            error_linear = self.person_dist - DIST_THRESHOLD  # Error is the distance between the robot and the person
            error_angular = self.person_angle  # Error is the angle between the robot and the person
            linear_vel = Kp_linear * error_linear  # Calculate the linear velocity proportional to the error
            angular_vel = Kp_angular * error_angular  # Calculate the angular velocity proportional to the error
            print(linear_vel, angular_vel)
            self.drive(linear_vel=linear_vel, angular_vel=angular_vel)
            print("moving")
            
        else:
            # No person detected, stop driving
            self.drive(linear_vel=0.0, angular_vel=0.0)
            print("no person")

    def drive(self, linear_vel, angular_vel):
        """ Send a drive command to the robot """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)      # Initialize communication with ROS
    node = PersonDetectionNode()   # Create our Node
    rclpy.spin(node)           # Run the Node until ready to shutdown
    node.destroy_node()        # Cleanup resources
    rclpy.shutdown()           # cleanup

if __name__ == '__main__':
    main()
