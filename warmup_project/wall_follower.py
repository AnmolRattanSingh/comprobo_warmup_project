
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from time import sleep
import time
import math

class WallFollowNode(Node):
    def __init__(self):
        """
        Initializes the ROS node, sets up a timer for run_loop, and creates 
        publishers and subscribers. Initializes scan and odometer readings as well as a flag.
        """
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
        """
        Callback function for the odometer subscriber that updates the odometer reading.
        """
        self.odom = msg

    def scan_callback(self, msg):
        """
        Callback function for the scan subscriber that updates the lidar scan data.
        """
        self.scan = msg

    def run_loop(self):
        """
        Main loop that controls the robot's behavior based on lidar scan and odometer readings.
        """
        # Check if scan messages are available
        # print system time
        if self.scan is not None:
            # Get the distances to the left and right walls
            fd = self.scan.ranges[0]
            d1 = self.scan.ranges[45]
            d2 = self.scan.ranges[135]
            # bd = self.scan.ranges[180]
            
            theta = math.acos((d1+d2)/math.sqrt(2*(d1**2+d2**2)))
            # turn amount theta
            # turn left
            # theta into degrees
            theta = (theta * 180) / math.pi
            print("theta: ", theta)
            print(f"({fd}, {d1}, {d2})")
            
            if fd < 1:
                self.turn_left_deg(-1.0, 90)
                sleep(2)
                if theta > 5:
                    if d1 < d2:
                        self.turn_left_deg(-1.0, theta)
                    else:
                        self.turn_left_deg(1.0, theta)
                    sleep(2)
            elif theta > 5:
                print("theta: ", theta)
                if d1 < d2:
                    self.turn_left_deg(-1.0, theta)
                else:
                    self.turn_left_deg(1.0, theta)
                sleep(2)
            else:
                self.drive_forward(0.5, 0.5)

    def run_loop2(self):
        """
        An alternative loop for control. Uses trigonometry to calculate the initial angle fix.
        """
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

    def turn_left_deg(self, angular_vel, degrees):
        """
        Turns the robot by a certain number of degrees at a given angular velocity.
        """
        ACC_CONST = 0.0
        self.drive(0.0, angular_vel)
        turn_radians = (degrees * math.pi) / 180.0
        sleep(abs(turn_radians / angular_vel) + ACC_CONST)
        self.drive(0.0, 0.0)
        if angular_vel < 0:
            print(f"turned right {degrees} degrees")
        else:
            print(f"turned left {degrees} degrees")

    def drive_forward(self, linear_vel, distance):
        """
        Drives the robot forward for a certain distance at a given linear velocity.
        """
        self.drive(linear_vel, 0.0)
        sleep(distance / linear_vel)
        self.drive(0.0, 0.0)

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
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
