
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

ACC_ERROR = 0.15
DRIVE_VEL: float = 0.25

class ObstacleAvoiderNode(Node):
    def __init__(self):
        """
        Initializes the ROS node, sets up publishers and subscribers, 
        and initializes several flags to track the robot's state. 
        Starts moving the robot forward.
        """
        super().__init__('obstacle_avoider_node')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.avoiding = False
        self.avoiding_right = False
        self.turning = False
        self.run_once(1, self.drive)

    def run_once(self, delay_s: float, fn, args: list = []):
        """
        Runs a given function once after a specified delay.
        """
        run_once_timer = self.create_timer(delay_s, lambda: (run_once_timer.cancel(), fn(*args)))

    def drive(self, linear_vel: float = DRIVE_VEL, angular_vel: float = 0.0):
        """
        Publishes a Twist message to control the robot's linear and angular velocity.
        """
        linear_vel = linear_vel if type(linear_vel) == float else float(linear_vel)
        angular_vel = angular_vel if type(angular_vel) == float else float(angular_vel)
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.vel_publisher.publish(msg)

    def turn_degrees(self, angular_vel: float, degrees: float, turn_right: bool = False, callback = None):
        """
        Turns the robot by a certain number of degrees at a given angular velocity. 
        It can turn either left or right.
        """
        self.turning = True
        if turn_right:
            angular_vel *= -1
        self.drive(0.0, angular_vel)
        turn_radians = (degrees * math.pi) / 180.0
        turn_time = abs(turn_radians / angular_vel) + ACC_ERROR
        def on_turn_done():
            self.drive(0.0, 0.0)
            self.turning = False
            if callback:
                callback()
        self.run_once(turn_time, on_turn_done)

    def avoid(self, turn_right: bool):
        """
        Implements the logic for avoiding obstacles based on sensor readings.
        """
        self.avoiding = True
        self.avoiding_right = turn_right
        self.drive()

    def on_scan(self, msg: LaserScan):
        """
        Callback function for the laser scan subscriber. It checks for obstacles 
        and decides what action to take.
        """
        front_distance = min(min(msg.ranges[315:360]), min(msg.ranges[0:45]), 10)
        right_distance = min(msg.ranges[45:135], default=10)
        left_distance = min(msg.ranges[225:315], default=10)

        if self.turning:
            return

        if front_distance < 0.75:
            self.drive(0.0)

        if self.avoiding:
            distance_side = msg.ranges[90] if self.avoiding_right else msg.ranges[270]
            def on_avoid_done():
                self.avoiding = False
                self.avoiding_right = False
                self.drive(DRIVE_VEL, 0.0)
            if distance_side > 1:
                self.avoiding = False
                def run_later():
                    self.turn_degrees(0.5, 90.0, not self.avoiding_right, on_avoid_done)
                self.run_once(.5, run_later)
                return
            else:
                self.drive(DRIVE_VEL, 0.0)
            return

        if front_distance < 0.75:
            self.drive(0.0)
            turn_right = right_distance > left_distance
            self.turn_degrees(1.0, 90.0, turn_right, lambda: self.avoid(turn_right))
        else:
            pass

def main(args=None):
    """
    Initializes ROS, creates the node, and spins it until shutdown.
    """
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
