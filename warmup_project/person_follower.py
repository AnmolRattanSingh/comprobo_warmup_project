
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
        """
        Initializes the ROS node, sets up a timer for the run_loop, and sets up 
        publishers and subscribers. Initializes variables for storing lidar scans 
        and person detection data.
        """
        super().__init__('send_message_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.person_angle = None
        self.person_dist = None
        self.scan = None

    def get_lidar_coords(self, msg):
        """
        Processes the lidar scan data to extract coordinate points based on lidar readings.
        """
        theta = 0
        self.scan = []
        for distance in msg.ranges:
            if distance !=0 and not math.isinf(distance):
                range_coords = (math.cos(theta) * distance, math.sin(theta) * distance)
                self.scan.append(range_coords)
            theta += msg.angle_increment

    def on_scan(self, msg):
        """
        Callback function for the lidar subscriber. Gets lidar coordinates, 
        calculates the centroid, and updates person detection variables.
        """
        self.get_lidar_coords(msg)
        if self.scan:
            cx_sum = 0.0
            cy_sum = 0.0
            total_weight = 0.0
            
            for coords in self.scan:
                distance = np.sqrt(np.sum(np.array(coords) ** 2))
                weight = 1.0 / (distance**2 + 1.0)  # Higher weight for points closer to the origin
                
                cx_sum += coords[0] * weight
                cy_sum += coords[1] * weight
                total_weight += weight

            if total_weight > 0.0:
                cx = cx_sum / total_weight
                cy = cy_sum / total_weight

                self.person_dist = np.sqrt(cx**2 + cy**2)
                self.person_angle = math.atan2(cy, cx)

                if cx < 0:
                    self.person_angle = -self.person_angle
                    self.person_dist = -self.person_dist

                print(self.person_dist, self.person_angle)


    def run_loop(self):
        """
        Main loop that controls the robot's movement. Uses proportional control 
        to follow the person based on the detected distance and angle.
        """
        if self.person_dist is not None:
            Kp_linear = 0.7
            Kp_angular = 1.0
            error_linear = self.person_dist - DIST_THRESHOLD
            error_angular = self.person_angle
            linear_vel = Kp_linear * error_linear
            angular_vel = Kp_angular * error_angular
            print(linear_vel, angular_vel)
            self.drive(linear_vel=linear_vel, angular_vel=angular_vel)
            print("moving")
        else:
            self.drive(linear_vel=0.0, angular_vel=0.0)
            print("no person")

    def drive(self, linear_vel, angular_vel):
        """
        Publishes a Twist message to control the robot's linear and angular velocity.
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)

def main(args=None):
    """
    Initializes ROS, creates the node, and spins it until shutdown.
    """
    rclpy.init(args=args)
    node = PersonDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
