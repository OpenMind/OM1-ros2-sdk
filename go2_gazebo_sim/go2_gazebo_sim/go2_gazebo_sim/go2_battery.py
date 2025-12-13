#!/usr/bin/env python3
"""Simulated battery publisher - auto-detects movement from cmd_vel"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from unitree_go.msg import LowState


class MockBatteryPublisher(Node):
    def __init__(self):
        super().__init__("mock_battery_publisher")

        # Parameters
        self.declare_parameter("soc", 80.0)
        self.declare_parameter("drain_rate", 0.5)
        self.declare_parameter("idle_drain_rate", 0.05)
        self.declare_parameter("charge_rate", 2.0)
        self.declare_parameter("is_charging", False)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("velocity_threshold", 0.01)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        # State
        self.cmd_vel_moving = False
        self.last_cmd_vel_time = self.get_clock().now()

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Subscribers
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        # Publisher
        self.pub = self.create_publisher(LowState, "/lf/lowstate", 10)

        # Timer
        publish_rate = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(1.0 / publish_rate, self.update_and_publish)

        self.get_logger().info("Mock Battery Publisher Started")
        self.get_logger().info("  Output: /lf/lowstate")
        self.get_logger().info("  Listens: /cmd_vel")

    def parameter_callback(self, params):
        for param in params:
            if param.name == "is_charging":
                status = "STARTED" if param.value else "STOPPED"
                self.get_logger().info(f"Charging {status}")
            elif param.name == "soc":
                self.get_logger().info(f"SoC manually set to {param.value:.1f}%")

        return SetParametersResult(successful=True)

    def cmd_vel_callback(self, msg: Twist):
        threshold = self.get_parameter("velocity_threshold").value
        linear_speed = (msg.linear.x**2 + msg.linear.y**2) ** 0.5
        angular_speed = abs(msg.angular.z)

        self.cmd_vel_moving = linear_speed > threshold or angular_speed > threshold
        self.last_cmd_vel_time = self.get_clock().now()

    def update_and_publish(self):
        soc = self.get_parameter("soc").value
        is_charging = self.get_parameter("is_charging").value
        timeout = self.get_parameter("cmd_vel_timeout").value

        # Check if moving
        time_since_cmd = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        is_moving = self.cmd_vel_moving and time_since_cmd < timeout

        if is_charging:
            is_moving = False

        # Update SoC
        if is_charging:
            soc = min(100.0, soc + self.get_parameter("charge_rate").value)
            current_ma = 3000
        elif is_moving:
            soc = max(0.0, soc - self.get_parameter("drain_rate").value)
            current_ma = -5000
        else:
            soc = max(0.0, soc - self.get_parameter("idle_drain_rate").value)
            current_ma = -500

        self.set_parameters([rclpy.parameter.Parameter("soc", value=soc)])

        # Publish
        msg = LowState()
        msg.bms_state.soc = int(soc)
        msg.bms_state.current = current_ma
        msg.power_v = 24.0 + (soc / 100.0) * 9.6
        msg.power_a = current_ma / 1000.0

        self.pub.publish(msg)

        status = "CHARGING" if is_charging else ("MOVING" if is_moving else "IDLE")
        self.get_logger().info(f"{status} | SoC: {soc:.1f}% | Current: {current_ma}mA")


def main(args=None):
    rclpy.init(args=args)
    node = MockBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()