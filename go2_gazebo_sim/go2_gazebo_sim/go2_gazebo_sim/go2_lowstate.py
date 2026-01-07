#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from unitree_go.msg import BmsState, IMUState, LowState, MotorState


class Go2LowStateNode(Node):
    """
    A ROS2 node that publishes mock LowState messages for Unitree Go2 simulation.
    This provides simulated robot state data including IMU, motor, and battery information.
    Battery drain is automatically calculated based on cmd_vel activity.
    """

    # Expected joint names from Gazebo
    EXPECTED_JOINT_NAMES = [
        "rf_hip_joint",
        "lf_lower_leg_joint",
        "rf_lower_leg_joint",
        "lf_upper_leg_joint",
        "rh_hip_joint",
        "rf_upper_leg_joint",
        "rh_upper_leg_joint",
        "rh_lower_leg_joint",
        "lh_hip_joint",
        "lf_hip_joint",
        "lh_upper_leg_joint",
        "lh_lower_leg_joint",
    ]

    # Mapping from Gazebo joint order to Unitree convention:
    # [rf_hip, rf_upper, rf_lower, lf_hip, lf_upper, lf_lower,
    #  rh_hip, rh_upper, rh_lower, lh_hip, lh_upper, lh_lower]
    UNITREE_JOINT_INDICES = [0, 5, 2, 9, 3, 1, 4, 6, 7, 8, 10, 11]

    def __init__(self):
        super().__init__("go2_lowstate_node")

        # Declare battery simulation parameters
        self.declare_parameter("soc", 100.000)
        self.declare_parameter("drain_rate", 0.001)
        self.declare_parameter("idle_drain_rate", 0.001)
        self.declare_parameter("charge_rate", 0.05)
        self.declare_parameter("is_charging", False)
        self.declare_parameter("velocity_threshold", 0.01)
        self.declare_parameter("cmd_vel_timeout", 0.5)

        # Publishers
        self.lowstate_publisher = self.create_publisher(LowState, "/lowstate", 10)
        self.lf_lowstate_publisher = self.create_publisher(LowState, "/lf/lowstate", 10)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        # Timer for publishing at 100Hz
        self.timer = self.create_timer(0.01, self.publish_lowstate)

        # State variables
        self.tick_counter = 0
        self.latest_joint_state = None
        self.cmd_vel_moving = False
        self.last_cmd_vel_time = self.get_clock().now()

        # Parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info("LowState Mock Node initialized")
        self.get_logger().info("Publishing to: lowstate and lf/lowstate at 100Hz")
        self.get_logger().info("Subscribing to: joint_states, cmd_vel")
        self.get_logger().info(
            f"Battery: SoC={self.get_parameter('soc').value}%, "
            f"drain_rate={self.get_parameter('drain_rate').value}%/tick"
        )

    def parameter_callback(self, params) -> SetParametersResult:
        """
        Callback for handling parameter changes at runtime.

        Parameters
        ----------
        params : list
            List of parameter changes

        Returns
        -------
        SetParametersResult
            Result indicating success or failure
        """
        for param in params:
            if param.name == "is_charging":
                status = "STARTED" if param.value else "STOPPED"
                self.get_logger().debug(f"Charging {status}")
            elif param.name == "soc":
                self.get_logger().debug(f"SoC manually set to {param.value:.1f}%")

        return SetParametersResult(successful=True)

    def joint_state_callback(self, msg: JointState):
        """
        Callback function for joint state messages from Gazebo.

        Parameters
        ----------
        msg : JointState
            Joint state message containing positions and velocities
        """
        self.latest_joint_state = msg

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback function for velocity commands to detect robot movement.

        Parameters
        ----------
        msg : Twist
            Velocity command message
        """
        threshold = self.get_parameter("velocity_threshold").value
        linear_speed = (msg.linear.x**2 + msg.linear.y**2) ** 0.5
        angular_speed = abs(msg.angular.z)

        self.cmd_vel_moving = linear_speed > threshold or angular_speed > threshold
        self.last_cmd_vel_time = self.get_clock().now()

    def get_unitree_joint_data(self):
        """
        Convert Gazebo joint states to Unitree motor order.

        Returns
        -------
        tuple
            (positions, velocities) in Unitree order, or (None, None) if no data available
        """
        if self.latest_joint_state is None:
            return None, None

        name_to_index = {name: i for i, name in enumerate(self.latest_joint_state.name)}

        missing_joints = [
            name for name in self.EXPECTED_JOINT_NAMES if name not in name_to_index
        ]
        if missing_joints:
            self.get_logger().warn(f"Missing joints: {missing_joints}")
            return None, None

        gazebo_positions = []
        gazebo_velocities = []

        for joint_name in self.EXPECTED_JOINT_NAMES:
            idx = name_to_index[joint_name]
            gazebo_positions.append(self.latest_joint_state.position[idx])
            gazebo_velocities.append(
                self.latest_joint_state.velocity[idx]
                if idx < len(self.latest_joint_state.velocity)
                else 0.0
            )

        unitree_positions = [gazebo_positions[i] for i in self.UNITREE_JOINT_INDICES]
        unitree_velocities = [gazebo_velocities[i] for i in self.UNITREE_JOINT_INDICES]

        return unitree_positions, unitree_velocities

    def update_battery_state(self):
        """
        Update battery state based on robot activity.

        Returns
        -------
        tuple
            (soc, current_ma, is_moving) - current battery state
        """
        soc = self.get_parameter("soc").value
        is_charging = self.get_parameter("is_charging").value
        timeout = self.get_parameter("cmd_vel_timeout").value

        # Check if robot is moving based on cmd_vel timeout
        time_since_cmd = (
            self.get_clock().now() - self.last_cmd_vel_time
        ).nanoseconds / 1e9
        is_moving = self.cmd_vel_moving and time_since_cmd < timeout

        # Don't drain while charging
        if is_charging:
            is_moving = False

        # Update SoC based on activity
        if is_charging:
            soc = min(100.0, soc + self.get_parameter("charge_rate").value)
            current_ma = 3000  # Charging current
        elif is_moving:
            soc = max(0.0, soc - self.get_parameter("drain_rate").value)
            current_ma = -5000  # High drain while moving
        else:
            soc = max(0.0, soc - self.get_parameter("idle_drain_rate").value)
            current_ma = -500  # Low idle drain

        # Update the parameter with new SoC value
        self.set_parameters([Parameter("soc", value=soc)])

        return soc, current_ma, is_moving

    def create_lowstate_message(self) -> LowState:
        """
        Create a LowState message with current robot state.

        Returns
        -------
        LowState
            A populated LowState message with current data
        """
        msg = LowState()

        # Header information
        msg.head = [254, 239]
        msg.level_flag = 0
        msg.frame_reserve = 0
        msg.sn = [0, 0]
        msg.version = [0, 0]
        msg.bandwidth = 0

        # IMU State
        msg.imu_state = IMUState()
        msg.imu_state.quaternion = [0.0, 0.0, 0.0, 0.0]
        msg.imu_state.gyroscope = [0.0, 0.0, 0.0]
        msg.imu_state.accelerometer = [0.0, 0.0, 0.0]
        msg.imu_state.rpy = [0.0, 0.0, 0.0]
        msg.imu_state.temperature = 0

        # Motor States (20 motors total, 12 active + 8 inactive)
        unitree_positions, unitree_velocities = self.get_unitree_joint_data()

        # Use default values if no joint data is available yet
        if unitree_positions is None:
            unitree_positions = [0.0] * 12
            unitree_velocities = [0.0] * 12

        # Create motor state array with exactly 20 elements
        motor_states = []
        for i in range(20):
            motor = MotorState()
            if i < 12:
                motor.mode = 1
                motor.q = unitree_positions[i]
                motor.dq = unitree_velocities[i]
                motor.ddq = 0.0
                motor.tau_est = 0.025 + (i % 3) * 0.01  # Small varying torque estimates
                motor.q_raw = 0.0
                motor.dq_raw = 0.0
                motor.ddq_raw = 0.0
                motor.temperature = 26 + (i % 6)  # Varying temps 26-31°C
                motor.lost = 0
                motor.reserve = [0, 588]
            else:
                motor.mode = 0
                motor.q = 0.0
                motor.dq = 0.0
                motor.ddq = 0.0
                motor.tau_est = 0.0
                motor.q_raw = 0.0
                motor.dq_raw = 0.0
                motor.ddq_raw = 0.0
                motor.temperature = 0
                motor.lost = 0
                motor.reserve = [0, 0]

            motor_states.append(motor)

        msg.motor_state = motor_states

        # Battery State (dynamic based on movement)
        soc, current_ma, is_moving = self.update_battery_state()

        msg.bms_state = BmsState()
        msg.bms_state.version_high = 1
        msg.bms_state.version_low = 18
        msg.bms_state.status = 8
        msg.bms_state.soc = int(soc)
        msg.bms_state.current = current_ma
        msg.bms_state.cycle = 5
        msg.bms_state.bq_ntc = [25, 23]
        msg.bms_state.mcu_ntc = [29, 28]
        msg.bms_state.cell_vol = [
            3654,
            3663,
            3664,
            3663,
            3662,
            3662,
            3663,
            3653,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]

        # Foot force sensors
        msg.foot_force = [21, 22, 22, 22]
        msg.foot_force_est = [0, 0, 0, 0]

        # Tick counter (incrementing)
        msg.tick = self.tick_counter
        self.tick_counter += 1

        # Wireless remote (all zeros)
        msg.wireless_remote = [0] * 40

        # Power state (calculated from SoC)
        msg.power_v = 24.0 + (soc / 100.0) * 9.6  # ~24V at 0%, ~33.6V at 100%
        msg.power_a = current_ma / 1000.0

        # Additional fields
        msg.bit_flag = 36
        msg.adc_reel = 0.0032226562034338713
        msg.temperature_ntc1 = 43
        msg.temperature_ntc2 = 40
        msg.fan_frequency = [0, 0, 0, 0]
        msg.reserve = 0
        msg.crc = 1036487475

        return msg

    def publish_lowstate(self):
        """
        Timer callback to publish LowState messages at regular intervals.
        """
        msg = self.create_lowstate_message()
        self.lowstate_publisher.publish(msg)
        self.lf_lowstate_publisher.publish(msg)

        # Log every 500 messages (5 seconds at 100Hz)
        if self.tick_counter % 500 == 0:
            joint_status = (
                "No joint data" if self.latest_joint_state is None else "Joint data OK"
            )
            is_charging = self.get_parameter("is_charging").value
            status = (
                "CHARGING"
                if is_charging
                else ("MOVING" if self.cmd_vel_moving else "IDLE")
            )
            self.get_logger().info(
                f"[{status}] SoC: {msg.bms_state.soc}%, "
                f"Current: {msg.bms_state.current}mA, "
                f"Joints: {joint_status}"
            )


def main(args=None):
    """
    Main entry point for the go2_lowstate_node.
    """
    rclpy.init(args=args)

    node = None
    try:
        node = Go2LowStateNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        print(f"LowStateNode encountered an error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
