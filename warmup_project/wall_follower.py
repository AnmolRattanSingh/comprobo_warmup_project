
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
        # Implementation here (truncated for brevity)

    def run_loop2(self):
        """
        An alternative loop for control (though it seems this may be outdated or not used).
        """
        # Implementation here (truncated for brevity)

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
