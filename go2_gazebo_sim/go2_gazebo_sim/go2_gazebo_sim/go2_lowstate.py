#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from unitree_go.msg import BmsState, IMUState, LowState, MotorState


class Go2LowStateNode(Node):
    """
    A ROS2 node that publishes mock LowState messages for Unitree Go2 simulation.
    This provides simulated robot state data including IMU, motor, and battery information.
    """

    def __init__(self):
        super().__init__("go2_lowstate_node")

        self.lowstate_publisher = self.create_publisher(LowState, "/lowstate", 10)

        self.joint_state_subscriber = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.timer = self.create_timer(0.01, self.publish_lowstate)

        self.tick_counter = 0

        self.latest_joint_state = None

        self.get_logger().info("LowState Mock Node initialized")
        self.get_logger().info("Publishing to: /lowstate at 100Hz")
        self.get_logger().info("Subscribing to: /joint_states")

    def joint_state_callback(self, msg: JointState):
        """
        Callback function for joint state messages from Gazebo.

        Parameters:
        -----------
        msg : JointState
            Joint state message containing positions and velocities
        """
        self.latest_joint_state = msg

    def get_unitree_joint_data(self):
        """
        Convert Gazebo joint states to Unitree motor order.

        Returns:
        --------
        tuple
            (positions, velocities) in Unitree order, or None if no data available
        """
        if self.latest_joint_state is None:
            return None, None

        # Expected joint names from Gazebo in this order:
        expected_names = [
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

        name_to_index = {name: i for i, name in enumerate(self.latest_joint_state.name)}

        missing_joints = [name for name in expected_names if name not in name_to_index]
        if missing_joints:
            self.get_logger().warn(f"Missing joints: {missing_joints}")
            return None, None

        gazebo_positions = []
        gazebo_velocities = []

        for joint_name in expected_names:
            idx = name_to_index[joint_name]
            gazebo_positions.append(self.latest_joint_state.position[idx])
            gazebo_velocities.append(
                self.latest_joint_state.velocity[idx]
                if idx < len(self.latest_joint_state.velocity)
                else 0.0
            )

        # Reorder to Unitree convention: [rf_hip, rf_upper, rf_lower, lf_hip, lf_upper, lf_lower,
        #                                rh_hip, rh_upper, rh_lower, lh_hip, lh_upper, lh_lower]
        # Mapping from gazebo order to unitree order:
        unitree_indices = [0, 5, 2, 9, 3, 1, 4, 6, 7, 8, 10, 11]

        unitree_positions = [gazebo_positions[i] for i in unitree_indices]
        unitree_velocities = [gazebo_velocities[i] for i in unitree_indices]

        return unitree_positions, unitree_velocities

    def create_mock_lowstate(self) -> LowState:
        """
        Create a mock LowState message based on the provided sample data.

        Returns:
        --------
        LowState
            A populated LowState message with mock data
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
        # Get current joint data from Gazebo
        unitree_positions, unitree_velocities = self.get_unitree_joint_data()

        # Use default values if no joint data is available yet
        if unitree_positions is None:
            unitree_positions = [0.0] * 12
            unitree_velocities = [0.0] * 12

        # Motor data with realistic temperatures and small torque estimates
        motor_data = []
        for i in range(12):
            motor_data.append(
                {
                    "mode": 1,
                    "q": unitree_positions[i],
                    "dq": unitree_velocities[i],
                    "ddq": 0.0,
                    "tau_est": 0.025 + (i % 3) * 0.01,  # Small varying torque estimates
                    "q_raw": 0.0,
                    "dq_raw": 0.0,
                    "ddq_raw": 0.0,
                    "temperature": 26 + (i % 6),  # Varying temps 26-31°C
                    "lost": 0,
                    "reserve": [0, 588],
                }
            )

        # Create motor state array with exactly 20 elements
        motor_states = []
        for i in range(20):
            motor = MotorState()
            if i < 12:
                data = motor_data[i]
                motor.mode = data["mode"]
                motor.q = data["q"]
                motor.dq = data["dq"]
                motor.ddq = data["ddq"]
                motor.tau_est = data["tau_est"]
                motor.q_raw = data["q_raw"]
                motor.dq_raw = data["dq_raw"]
                motor.ddq_raw = data["ddq_raw"]
                motor.temperature = data["temperature"]
                motor.lost = data["lost"]
                motor.reserve = data["reserve"]
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

        # BMS State
        msg.bms_state = BmsState()
        msg.bms_state.version_high = 1
        msg.bms_state.version_low = 18
        msg.bms_state.status = 8
        msg.bms_state.soc = 100
        msg.bms_state.current = -2752
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
            0,  # Remaining cells are 0
        ]

        # Foot force sensors
        msg.foot_force = [21, 22, 22, 22]
        msg.foot_force_est = [0, 0, 0, 0]

        # Tick counter (incrementing)
        msg.tick = self.tick_counter
        self.tick_counter += 1

        # Wireless remote (all zeros)
        msg.wireless_remote = [0] * 40

        # Additional fields
        msg.bit_flag = 36
        msg.adc_reel = 0.0032226562034338713
        msg.temperature_ntc1 = 43
        msg.temperature_ntc2 = 40
        msg.power_v = 28.67633056640625
        msg.power_a = 1.1279981136322021
        msg.fan_frequency = [0, 0, 0, 0]
        msg.reserve = 0
        msg.crc = 1036487475

        return msg

    def publish_lowstate(self):
        """
        Timer callback to publish LowState messages at regular intervals.
        """
        msg = self.create_mock_lowstate()
        self.lowstate_publisher.publish(msg)

        # Log every 100 messages (1 second at 100Hz)
        if self.tick_counter % 100 == 0:
            joint_status = (
                "No joint data" if self.latest_joint_state is None else "Joint data OK"
            )
            self.get_logger().debug(
                f"Published LowState - Tick: {msg.tick}, "
                f"Battery SOC: {msg.bms_state.soc}%, "
                f"IMU Temp: {msg.imu_state.temperature}°C, "
                f"Joint Status: {joint_status}"
            )


def main(args=None):
    """
    Main entry point for the lowstate_mock_node.
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
