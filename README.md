# Warmup Project

## Description

The objective of this project was to acquire a foundational grasp of ROS (Robot Operating System), explore the utilization of sensor data for real-time adjustments to robot behavior, and gain proficiency in diagnosing issues/debugging in the ROS platform.

## Robot Behavior

### Teleop

#### Behavior

The teleop node allows the user to control the robot using the keyboard. The robot can be moved forward, backward, and rotated left and right. The robot will continue to move in the direction of the last key pressed until another key is pressed. The robot will stop moving when the emergency stop key is pressed. Since we started with implementing this node, the main challenge was to get familiar with moving the neato. Our first approach was to simply publish to the `cmd_vel` topic and sleep for a fixed duration. 

### Drive in a Square

#### Behavior

Drives the robot in a 1m x 1m square. We solved the challenge using time based control borrowed from the `Teleop` node. Since we already had functions to move the neato a desired distance and be able to make turns, it was relatively straightforward to make it move in a square using a for loop.

#### Implementation

1. Node Initialization:

    - The DrawSquareNode class is defined, which inherits from the Node class provided by ROS.
    - In the constructor (`__init__`), the node is given the name 'send_message_node'.
    - A timer is created to call the run_loop function at a rate of 10 times per second (10 Hz).
    - A publisher (self.vel_publisher) is created to publish messages of type Twist to the `cmd_vel` topic. This topic is used to control the velocity of a robot.
    - A boolean flag self.square_done is initialized to False.

2. Run Loop (run_loop):

    - The run_loop function is called by the timer at 10 Hz.
    - It checks if self.square_done is False.
    - If not, it enters a loop that repeats four times, effectively instructing the robot to move forward and turn left to form a square.
    - It prints messages indicating that the robot is driving forward and turning left.
    - The drive_forward and turn_left_deg functions are called to control the robot's movements.

3. Drive Functions:

    - drive(self, linear_vel, angular_vel): This function creates a Twist message with linear and angular velocity values and publishes it to control the robot's motion.
    - turn_left_deg(self, angular_vel, degrees): This function makes the robot turn left by setting angular velocity and calculating the time needed for the turn based on the desired angle in degrees.
    - drive_forward(self, linear_vel, distance): This function makes the robot move forward by setting linear velocity and calculating the time needed to cover the desired distance.

4. Main Function (main):

    - The main function is the entry point of the script.
    - It initializes ROS using rclpy.init().
    - Creates an instance of the DrawSquareNode.
    - Enters the ROS spin loop to keep the node running until it's ready to shutdown.
    - Once finished, it cleans up the node and shuts down ROS gracefully.

Tried to check angle 135 and 45 first and make them equal distance to try and follow the wall. Then realized that I need to do math to actually make the angles 45. Use trig or smth.

Drive square tried first with timed driving and turns. Added random sleep times first 1, 0.5s. Then realized need to do math for turning time. Then realized exact math not ideal since it takes
some time to accelerate. Then added random constant about 0.09 to account for that. Better solution could be used. Teleop was pretty easy to implement. Have a goal to make it record key holds and do video game like driving mechanics.
