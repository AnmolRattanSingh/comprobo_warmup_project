import rclpy;
from rclpy.node import Node;
from geometry_msgs.msg import Twist;
from sensor_msgs.msg import LaserScan;

class ObstacleAvoiderNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.run_loop)
    
    def run_loop(self):
        self.drive(0.5, 0.0)

    def on_scan(self, msg: LaserScan):
        # check for objects in front of the robot
        # if there is an object in front of the robot, turn left 90 deg
        # if there is no object in front of the robot, drive forward

        front_distance = msg.ranges[0]
        left_distance = msg.ranges[90]
        right_distance = msg.ranges[270]

        print(f"all in one line: {front_distance}, {left_distance}, {right_distance}")