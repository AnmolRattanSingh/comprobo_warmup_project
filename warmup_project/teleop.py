import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import psutil

class Teleop(Node):
    def __init__(self):
        """
        Initializes the ROS node, sets up a timer for the run_loop, and creates 
        a publisher for velocity commands. It also sets terminal settings to get keyboard inputs.
        """
        super().__init__('teleop_node')
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

        self.child_process = None
        self.child_name = ""

        self.print_help()

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key
    
    def spawn_child(self, name: str):
        self.destroy_child()
        self.child_name = name
        self.child_process = subprocess.Popen(['ros2', 'run', 'warmup_project', name])


    def destroy_child(self):
        if self.child_process is None:
            return ""

        # kill all children of the child process
        parent = psutil.Process(self.child_process.pid)
        for child in parent.children(recursive=True):
            child.kill()

        parent.kill()

        self.child_process = None
        old_child = self.child_name
        self.child_name = ""
        self.drive(0.0, 0.0)
        return old_child

    def print_help(self):
        print("\nTeleop keys:")
        print("w: drive forward")
        print("s: drive backward")
        print("a: turn left")
        print("d: turn right")
        print("e: stop")
        print("p: person follower")
        print("o: obstacle avoider")
        print("i: wall follower")
        print("u: drive square")
        print(".: stop child node\n")
    
    def run_loop(self):
        """
        Main loop that reads keyboard inputs and sends the appropriate Twist 
        message to control the robot.
        """
        while self.key != '\x03':
            key = self.getKey()

            if key == '.':
                if self.child_process is None:
                    print("no child node to stop")
                    continue

                old_child = self.destroy_child()
                print(f"\nstopped {old_child}")

            if self.child_process is not None:
                print(f"\nchild node ({self.child_name}) running, press . to stop")
            # move forward if 'w' is pressed
            elif key == '.':
                pass
            elif key == 'w':
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
            # p = person follower
            elif key == 'p':
                self.spawn_child('person_follower')
                print("running person_follower! press . to stop")
            # o = obstacle avoider
            elif key == 'o':
                self.spawn_child('obstacle_avoider')
                print("running obstacle_avoider! press . to stop")
            # i = wall follower
            elif key == 'i':
                self.spawn_child('wall_follower')
                print("running wall_follower! press . to stop")
            # u = drive square
            elif key == 'u':
                self.spawn_child('drive_square')
                print("running drive_square! press . to stop")
            # h = help
            elif key == 'h':
                self.print_help()
            # otherwise, do nothing
            else:
                print("invalid key, press h for help")
                

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
