#!/usr/bin/env python3

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from unitree_api.msg import (
    Request,
    RequestIdentity,
    Response,
    ResponseHeader,
    ResponseStatus,
)


class Go2SportNode(Node):
    """
    A ROS2 node that converts Unitree Go2 sport API requests to cmd_vel (geometry_msgs/Twist) messages
    for robot movement control in simulation. This is the reverse of the go2_movement.py functionality.
    """

    def __init__(self):
        super().__init__("go2_sport_node")

        self.SPORT_API_ID_MOVE = 1008
        self.SPORT_API_ID_BALANCESTAND = 1002
        self.SPORT_API_ID_STOPMOVE = 1003

        self.sport_subscriber = self.create_subscription(
            Request, "/api/sport/request", self.sport_request_callback, 10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        self.sport_response_publisher = self.create_publisher(
            Response, "api/sport/response", 10
        )

        self.get_logger().info("Sport to CmdVel Node initialized")
        self.get_logger().info("Subscribing to: api/sport/request")
        self.get_logger().info("Publishing to: cmd_vel")
        self.get_logger().info("Publishing to: api/sport/response")

    def sport_request_callback(self, msg: Request):
        """
        Callback function for sport request messages.
        Converts Unitree sport commands to Twist messages and sends response.

        Parameters
        ----------
        msg : unitree_api.msg.Request
            The incoming sport request message containing movement commands.
        """
        api_id = msg.header.identity.api_id

        self.get_logger().debug(f"Received sport request with API ID: {api_id}")

        response_msg = Response()
        response_msg.header = ResponseHeader()
        response_msg.header.identity = RequestIdentity()
        response_msg.header.identity.api_id = api_id
        response_msg.header.status = ResponseStatus()

        try:
            if api_id == self.SPORT_API_ID_MOVE:
                self.handle_move_command(msg.parameter)
                response_msg.header.status.code = 0  # Success
                response_msg.data = "Move command executed"
                self.get_logger().debug("Processed move command")

            elif api_id == self.SPORT_API_ID_STOPMOVE:
                self.handle_stop_command()
                response_msg.header.status.code = 0  # Success
                response_msg.data = "Stop command executed"
                self.get_logger().debug("Processed stop command")

            elif api_id == self.SPORT_API_ID_BALANCESTAND:
                self.handle_balance_stand_command()
                response_msg.header.status.code = 0  # Success
                response_msg.data = "Balance stand command executed"
                self.get_logger().debug("Processed balance stand command")

            else:
                self.get_logger().warn(f"Unknown sport API ID: {api_id}")
                response_msg.header.status.code = -1  # Error
                response_msg.data = f"Unknown API ID: {api_id}"

        except Exception as e:
            self.get_logger().error(f"Error processing sport command: {e}")
            response_msg.header.status.code = -1  # Error
            response_msg.data = f"Error: {str(e)}"

        self.sport_response_publisher.publish(response_msg)

    def handle_move_command(self, parameter: str):
        """
        Handle move command by parsing parameters and publishing cmd_vel.

        Parameters
        ----------
        parameter : str
            JSON string containing movement parameters (x, y, z)
        """
        if not parameter.strip():
            self.get_logger().warn("Empty parameter for move command")
            return

        try:
            move_params = json.loads(parameter)

            linear_x = float(move_params.get("x", 0.0))
            linear_y = float(move_params.get("y", 0.0))
            angular_z = float(move_params.get("z", 0.0))

            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = linear_y
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = angular_z

            self.cmd_vel_publisher.publish(twist_msg)

            self.get_logger().debug(
                f"Published cmd_vel: vx={linear_x:.3f}, vy={linear_y:.3f}, vyaw={angular_z:.3f}"
            )

        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Failed to parse move parameters: {e}")
            raise

    def handle_stop_command(self):
        """
        Handle stop command by publishing zero velocity cmd_vel.
        """
        twist_msg = Twist()

        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().debug("Published stop cmd_vel (all zeros)")

    def handle_balance_stand_command(self):
        """
        Handle balance stand command.
        For simulation, this just ensures the robot is stopped.
        """
        self.handle_stop_command()
        self.get_logger().debug("Processed balance stand as stop command")


def main(args=None):
    """
    Main entry point for the go2_sport_action.
    """
    rclpy.init(args=args)

    node = None
    try:
        node = Go2SportNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        print(f"Go2SportNode encountered an error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
