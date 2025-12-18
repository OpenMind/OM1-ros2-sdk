#!/usr/bin/env python3
"""
Joy Robot Switcher Node

Allows a single joystick to control multiple robots by switching between them.
Uses button presses to switch the active robot.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyRobotSwitcher(Node):
    """
    A ROS2 node that allows switching joystick control between multiple robots.
    
    Subscribes to /joy and republishes cmd_vel to the selected robot's namespace.
    Button presses switch between robots.
    """

    def __init__(self):
        super().__init__("joy_robot_switcher")

        # Parameters
        self.declare_parameter("num_robots", 2)
        self.declare_parameter("robot1_button", 4)       # LB on Xbox controller = robot1
        self.declare_parameter("robot2_button", 5)       # RB on Xbox controller = robot2
        self.declare_parameter("linear_axis", 1)         # Left stick Y
        self.declare_parameter("angular_axis", 0)        # Left stick X
        self.declare_parameter("linear_scale", 0.5)      # Max linear velocity
        self.declare_parameter("angular_scale", 1.0)     # Max angular velocity

        self.num_robots = self.get_parameter("num_robots").value
        self.robot1_button = self.get_parameter("robot1_button").value
        self.robot2_button = self.get_parameter("robot2_button").value
        self.linear_axis = self.get_parameter("linear_axis").value
        self.angular_axis = self.get_parameter("angular_axis").value
        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value

        # Subscribe to global joy topic
        self.joy_subscription = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10
        )

        # Create publishers for each robot's cmd_vel
        self.cmd_vel_publishers = {}
        for i in range(1, self.num_robots + 1):
            topic = f"/robot{i}/cmd_vel"
            self.cmd_vel_publishers[i] = self.create_publisher(Twist, topic, 10)
            self.get_logger().info(f"Created publisher for {topic}")

        self.get_logger().info(f"Joy Robot Switcher initialized")
        self.get_logger().info(f"Controlling {self.num_robots} robots")
        self.get_logger().info(f"Hold LB (button {self.robot1_button}) to control robot1")
        self.get_logger().info(f"Hold RB (button {self.robot2_button}) to control robot2")

    def joy_callback(self, msg: Joy):
        """
        Handle joystick input and route to the selected robot.
        LB held = control robot1, RB held = control robot2.
        
        Parameters:
        -----------
        msg : Joy
            Joystick message with button and axis data
        """
        # Convert joystick axes to cmd_vel
        twist = Twist()
        if len(msg.axes) > max(self.linear_axis, self.angular_axis):
            twist.linear.x = msg.axes[self.linear_axis] * self.linear_scale
            twist.angular.z = msg.axes[self.angular_axis] * self.angular_scale

        # Check which robot button is pressed
        robot1_active = (len(msg.buttons) > self.robot1_button and 
                         msg.buttons[self.robot1_button] == 1)
        robot2_active = (len(msg.buttons) > self.robot2_button and 
                         msg.buttons[self.robot2_button] == 1)

        # Publish to robot1 if LB held, otherwise stop
        if robot1_active:
            self.cmd_vel_publishers[1].publish(twist)
        else:
            self.cmd_vel_publishers[1].publish(Twist())  # Stop

        # Publish to robot2 if RB held, otherwise stop
        if robot2_active and self.num_robots >= 2:
            self.cmd_vel_publishers[2].publish(twist)
        elif self.num_robots >= 2:
            self.cmd_vel_publishers[2].publish(Twist())  # Stop


def main(args=None):
    """Main entry point for the joy_robot_switcher node."""
    rclpy.init(args=args)

    node = None
    try:
        node = JoyRobotSwitcher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        print(f"JoyRobotSwitcher encountered an error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
