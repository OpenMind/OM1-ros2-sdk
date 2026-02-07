# ruff: noqa: E402

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import argparse
import logging
import os
import time
from typing import Optional, Tuple

import carb
import numpy as np
import omni.appwindow  # Contains handle to keyboard
import utils as ros_utils
import yaml
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.rotations import quat_to_rot_matrix
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.policy.examples.controllers import PolicyController
from isaacsim.robot.policy.examples.controllers.config_loader import (
    get_action,
    get_observations,
    parse_env_config,
)
from isaacsim.storage.native import get_assets_root_path

DEFAULT_POLICY_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "checkpoints", "50000"
)
logger = logging.getLogger(__name__)


def _load_yaml(path: str) -> dict:
    try:
        with open(path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file) or {}
    except Exception:
        return {}


def _expand_param(value, size: int, default: float) -> np.ndarray:
    if value is None:
        return np.full(size, default, dtype=np.float32)
    if isinstance(value, (int, float)):
        return np.full(size, float(value), dtype=np.float32)
    arr = np.array(value, dtype=np.float32)
    if arr.size != size:
        return np.full(size, default, dtype=np.float32)
    return arr


def _resolve_command_limits(
    deploy_cfg: dict, env_cfg: dict
) -> Tuple[np.ndarray, np.ndarray]:
    def _from_cfg(cfg: dict):
        ranges = (
            cfg.get("commands", {}).get("base_velocity", {}).get("ranges")
            if cfg
            else None
        )
        if not isinstance(ranges, dict):
            return None

        def _pair(key: str, fallback: Tuple[float, float]):
            val = ranges.get(key)
            if isinstance(val, (list, tuple)) and len(val) == 2:
                try:
                    return float(val[0]), float(val[1])
                except Exception:
                    return fallback
            return fallback

        return np.array(
            [
                _pair("lin_vel_x", (-1.0, 1.0)),
                _pair("lin_vel_y", (-1.0, 1.0)),
                _pair("ang_vel_z", (-1.0, 1.0)),
            ],
            dtype=np.float32,
        )

    pairs = _from_cfg(deploy_cfg)
    if pairs is None:
        pairs = _from_cfg(env_cfg)
    if pairs is None:
        pairs = np.array([[-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0]], dtype=np.float32)

    cmd_min = pairs[:, 0]
    cmd_max = pairs[:, 1]
    return cmd_min, cmd_max


def _resolve_usd_path(env_cfg: dict) -> str:
    # First priority: check for local assets directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    local_usd_path = os.path.join(script_dir, "assets", "go2", "usd", "go2.usd")
    if os.path.isfile(local_usd_path):
        logger.info("Using local Go2 USD model: %s", local_usd_path)
        return local_usd_path

    # Second priority: check env.yaml usd_path
    usd_path = (
        env_cfg.get("scene", {}).get("robot", {}).get("spawn", {}).get("usd_path")
    )
    if usd_path:
        # If it's an absolute path that exists, use it
        if os.path.isabs(usd_path) and os.path.isfile(usd_path):
            logger.info("Using Go2 USD model from absolute path: %s", usd_path)
            return usd_path
        # If it's a relative path, try to resolve it from the script directory
        relative_path = os.path.join(script_dir, usd_path)
        if os.path.isfile(relative_path):
            logger.info("Using Go2 USD model from relative path: %s", relative_path)
            return relative_path
        # Otherwise use as-is (might be resolved by Isaac Sim)
        logger.warning("USD path from env.yaml not found: %s", usd_path)
        return usd_path

    # Fallback to Isaac Sim assets
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return ""
    fallback_path = assets_root_path + "/Isaac/Robots/Unitree/Go2/go2.usd"
    logger.info("Using Isaac Sim default Go2 USD model: %s", fallback_path)
    return fallback_path


def _configure_ros_utils_paths(robot_root: str) -> None:
    ros_utils.GO2_STAGE_PATH = robot_root
    ros_utils.IMU_PRIM = f"{robot_root}/base/imu_link"
    ros_utils.CAMERA_LINK_PRIM = f"{robot_root}/base/camera_link"
    ros_utils.REALSENSE_DEPTH_CAMERA_PRIM = f"{ros_utils.CAMERA_LINK_PRIM}/realsense_depth_camera"
    ros_utils.REALSENSE_RGB_CAMERA_PRIM = f"{ros_utils.CAMERA_LINK_PRIM}/realsense_rgb_camera"
    ros_utils.GO2_RGB_CAMERA_PRIM = f"{ros_utils.CAMERA_LINK_PRIM}/go2_rgb_camera"
    ros_utils.L1_LINK_PRIM = f"{robot_root}/base/lidar_l1_link"
    ros_utils.L1_LIDAR_PRIM = f"{ros_utils.L1_LINK_PRIM}/lidar_l1_rtx"
    ros_utils.VELO_BASE_LINK_PRIM = f"{robot_root}/base/velodyne_base_link"
    ros_utils.VELO_LASER_LINK_PRIM = f"{ros_utils.VELO_BASE_LINK_PRIM}/laser"
    ros_utils.VELO_LIDAR_PRIM = f"{ros_utils.VELO_LASER_LINK_PRIM}/velodyne_vlp16_rtx"


def _validate_policy_paths(policy_dir: str) -> Tuple[str, str, str]:
    policy_path = os.path.join(policy_dir, "exported", "policy.pt")
    env_path = os.path.join(policy_dir, "params", "env.yaml")
    deploy_path = os.path.join(policy_dir, "params", "deploy.yaml")

    missing = []
    if not os.path.isfile(policy_path):
        missing.append(policy_path)
    if not os.path.isfile(env_path):
        missing.append(env_path)

    if missing:
        for path in missing:
            carb.log_error(f"Missing required policy file: {path}")
        raise FileNotFoundError("Required policy files not found.")

    if not os.path.isfile(deploy_path):
        logger.warning(
            "deploy.yaml not found at %s; using env.yaml ranges only", deploy_path
        )

    return policy_path, env_path, deploy_path


class Go2VelocityPolicy(PolicyController):
    """The Unitree Go2 running a velocity tracking locomotion policy."""

    def __init__(
        self,
        prim_path: str,
        policy_path: str,
        env_path: str,
        root_path: Optional[str] = None,
        name: str = "go2",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:
        super().__init__(name, prim_path, root_path, usd_path, position, orientation)
        self.load_policy(policy_path, env_path)

        self._obs_order = [
            "base_ang_vel",
            "projected_gravity",
            "velocity_commands",
            "joint_pos_rel",
            "joint_vel_rel",
            "last_action",
        ]

        obs_cfg = get_observations(self.policy_env_params) or {}
        self._obs_scales = {}
        for name in self._obs_order:
            scale = obs_cfg.get(name, {}).get("scale")
            if scale is None:
                self._obs_scales[name] = 1.0
            elif isinstance(scale, (int, float)):
                self._obs_scales[name] = float(scale)
            else:
                self._obs_scales[name] = np.array(scale, dtype=np.float32)

        action_terms = get_action(self.policy_env_params) or {}
        self._action_cfg = next(iter(action_terms.values()), {})

        self._action_scale = None
        self._action_offset = None
        self._previous_action = None
        self._policy_counter = 0

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the robot controller with physics simulation view and control mode."""
        super().initialize(physics_sim_view=physics_sim_view, control_mode="position")
        dof_count = len(self.default_pos)
        self._action_scale = _expand_param(
            self._action_cfg.get("scale"), dof_count, default=1.0
        )
        if self._action_cfg.get("use_default_offset", False):
            self._action_offset = np.array(self.default_pos, dtype=np.float32)
        else:
            self._action_offset = _expand_param(
                self._action_cfg.get("offset"), dof_count, default=0.0
            )

        self._previous_action = np.zeros(dof_count, dtype=np.float32)
        self.action = np.zeros(dof_count, dtype=np.float32)

    def _compute_observation(self, command: np.ndarray) -> np.ndarray:
        ang_vel_I = self.robot.get_angular_velocity()
        _, q_IB = self.robot.get_world_pose()

        R_IB = quat_to_rot_matrix(q_IB)
        R_BI = R_IB.transpose()
        ang_vel_b = np.matmul(R_BI, ang_vel_I)
        gravity_b = np.matmul(R_BI, np.array([0.0, 0.0, -1.0]))

        current_joint_pos = self.robot.get_joint_positions()
        current_joint_vel = self.robot.get_joint_velocities()
        joint_pos_rel = current_joint_pos - self.default_pos

        obs = np.concatenate(
            [
                ang_vel_b * self._obs_scales["base_ang_vel"],
                gravity_b * self._obs_scales["projected_gravity"],
                command * self._obs_scales["velocity_commands"],
                joint_pos_rel * self._obs_scales["joint_pos_rel"],
                current_joint_vel * self._obs_scales["joint_vel_rel"],
                self._previous_action * self._obs_scales["last_action"],
            ],
            axis=0,
        ).astype(np.float32)
        return obs

    def forward(self, dt: float, command: np.ndarray) -> None:
        """Execute one forward step of the policy with the given command."""
        if self._policy_counter % self._decimation == 0:
            obs = self._compute_observation(command)
            self.action = np.array(self._compute_action(obs), dtype=np.float32)
            self._previous_action = self.action.copy()

        target_pos = self._action_offset + (self._action_scale * self.action)
        action = ArticulationAction(joint_positions=target_pos)
        self.robot.apply_action(action)
        self._policy_counter += 1


class Go2RosRunner(object):
    """Runner class for Go2 robot with ROS2 integration and sensor support."""

    def __init__(
        self,
        physics_dt: float,
        render_dt: float,
        policy_dir: str,
        cmd_vel_topic: str,
        vx_max: float,
        vy_max: float,
        wz_max: float,
        robot_root: str,
        cmd_vel_only: bool,
        enable_sensors: bool,
        enable_keyboard: bool,
    ) -> None:
        """
        Creates the simulation world with preset physics_dt and render_dt and creates a go2 robot inside the warehouse.

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """
        policy_path, env_path, deploy_path = _validate_policy_paths(policy_dir)

        env_cfg = parse_env_config(env_path)
        deploy_cfg = _load_yaml(deploy_path)

        usd_path = _resolve_usd_path(env_cfg)
        init_pos = np.array(
            env_cfg.get("scene", {})
            .get("robot", {})
            .get("init_state", {})
            .get("pos", (0.0, 0.0, 0.4))
        )

        self._world = World(
            stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt
        )
        self._physics_dt = physics_dt
        self._render_dt = render_dt

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise RuntimeError("Could not find Isaac Sim assets folder")

        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = (
            assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        )
        prim.GetReferences().AddReference(asset_path)

        if not usd_path:
            raise RuntimeError("Go2 USD path could not be resolved")

        self._go2 = Go2VelocityPolicy(
            prim_path=robot_root,
            name="Go2",
            usd_path=usd_path,
            position=init_pos,
            policy_path=policy_path,
            env_path=env_path,
        )

        cmd_min, cmd_max = _resolve_command_limits(deploy_cfg, env_cfg)
        args_min = np.array([-vx_max, -vy_max, -wz_max], dtype=np.float32)
        args_max = np.array([vx_max, vy_max, wz_max], dtype=np.float32)
        self._cmd_min = np.maximum(cmd_min, args_min)
        self._cmd_max = np.minimum(cmd_max, args_max)

        self._cmd_vel_topic = cmd_vel_topic
        self._cmd_vel_only = cmd_vel_only
        self._enable_sensors = enable_sensors
        self._enable_keyboard = enable_keyboard

        self._vx_max = vx_max
        self._vy_max = vy_max
        self._wz_max = wz_max

        self._linear_attr = None
        self._angular_attr = None
        self._sensors = {}

        self._robot_root = robot_root
        _configure_ros_utils_paths(robot_root)

        self._base_command = np.zeros(3, dtype=np.float32)

        cmd_scale = np.maximum(np.abs(self._cmd_min), np.abs(self._cmd_max))
        vx, vy, wz = cmd_scale.tolist()
        self._input_keyboard_mapping = {
            "NUMPAD_8": [vx, 0.0, 0.0],
            "UP": [vx, 0.0, 0.0],
            "NUMPAD_2": [-vx, 0.0, 0.0],
            "DOWN": [-vx, 0.0, 0.0],
            "NUMPAD_6": [0.0, -vy, 0.0],
            "RIGHT": [0.0, -vy, 0.0],
            "NUMPAD_4": [0.0, vy, 0.0],
            "LEFT": [0.0, vy, 0.0],
            "NUMPAD_7": [0.0, 0.0, wz],
            "N": [0.0, 0.0, wz],
            "NUMPAD_9": [0.0, 0.0, -wz],
            "M": [0.0, 0.0, -wz],
        }
        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        """
        Set up keyboard listener and add physics callback.

        """
        if self._enable_keyboard:
            self._appwindow = omni.appwindow.get_default_app_window()
            if self._appwindow is not None:
                self._input = carb.input.acquire_input_interface()
                self._keyboard = self._appwindow.get_keyboard()
                self._sub_keyboard = self._input.subscribe_to_keyboard_events(
                    self._keyboard, self._sub_keyboard_event
                )
            else:
                logger.warning("No app window found; keyboard control disabled")
                self._enable_keyboard = False
        self._world.add_physics_callback(
            "go2_ros2_step", callback_fn=self.on_physics_step
        )

    def setup_ros(self) -> None:
        """Set up ROS2 nodes for command velocity and sensor publishers."""
        self._linear_attr, self._angular_attr = ros_utils.setup_cmd_vel_graph(
            self._cmd_vel_topic
        )
        if not self._enable_sensors:
            return

        for _ in range(10):
            self._world.step(render=False)

        self._sensors = ros_utils.setup_sensors_delayed(simulation_app)
        ros_utils.setup_ros_publishers(self._sensors, simulation_app)

        ros_utils.setup_depth_camerainfo_graph(
            simulation_app,
            topic="/camera/realsense2_camera_node/depth/camera_info",
            frame_id="realsense_depth_camera",
            width=480,
            height=270,
            fx=242.479,
            fy=242.479,
            cx=242.736,
            cy=133.273,
        )

        ros_utils.setup_odom_publisher(simulation_app)
        ros_utils.setup_color_camera_publishers(self._sensors, simulation_app)
        ros_utils.setup_color_camerainfo_graph(
            simulation_app,
            topic="/camera/realsense2_camera_node/color/camera_info",
            frame_id="realsense_depth_camera",
            width=480,
            height=270,
            fx=242.479,
            fy=242.479,
            cx=242.736,
            cy=133.273,
        )

        ros_utils.setup_joint_states_publisher(simulation_app)

    def _get_cmd_vel(self) -> Optional[np.ndarray]:
        if self._linear_attr is None or self._angular_attr is None:
            return None
        lin = self._linear_attr.get()
        ang = self._angular_attr.get()
        if lin is None or ang is None:
            return None

        vx = ros_utils.clamp(float(lin[0]), -self._vx_max, self._vx_max)
        vy = ros_utils.clamp(float(lin[1]), -self._vy_max, self._vy_max)
        wz = ros_utils.clamp(float(ang[2]), -self._wz_max, self._wz_max)
        return np.array([vx, vy, wz], dtype=np.float32)

    def _update_odom(self) -> None:
        try:
            pos_w, quat_wxyz = self._go2.robot.get_world_pose()
            lin_vel = self._go2.robot.get_linear_velocity()
            ang_vel = self._go2.robot.get_angular_velocity()
            quat_xyzw = [quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]]

            ros_utils.update_odom_tf(pos_w, quat_xyzw)
            ros_utils.update_odom(pos_w, quat_xyzw, lin_vel, ang_vel)
        except Exception:
            return

    def on_physics_step(self, step_size) -> None:
        """
        Physics call back, initialize robot (first frame) and call controller forward function.

        """
        if self.first_step:
            self._go2.initialize()
            self.first_step = False
            return
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
            self.first_step = True
            return

        cmd = self._base_command.copy()
        cmd_vel = self._get_cmd_vel()
        if cmd_vel is not None:
            if self._cmd_vel_only:
                cmd = cmd_vel
            else:
                cmd = cmd + cmd_vel

        cmd = np.minimum(np.maximum(cmd, self._cmd_min), self._cmd_max)
        self._go2.forward(step_size, cmd)
        self._update_odom()

    def run(self, real_time: bool) -> None:
        """
        Step simulation based on rendering downtime.

        """
        while simulation_app.is_running():
            t0 = time.time()
            self._world.step(render=True)
            if self._world.is_stopped():
                self.needs_reset = True
            if real_time:
                sleep = self._physics_dt - (time.time() - t0)
                if sleep > 0:
                    time.sleep(sleep)
        return

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """
        Keyboard subscriber callback to when kit is updated.

        """
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(
                    self._input_keyboard_mapping[event.input.name], dtype=np.float32
                )

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(
                    self._input_keyboard_mapping[event.input.name], dtype=np.float32
                )
        return True


def main():
    """
    Instantiate the Go2 runner with ROS2 + sensors.

    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy_dir", default=DEFAULT_POLICY_DIR)
    parser.add_argument("--cmd_vel_topic", default="/cmd_vel")
    parser.add_argument("--vx_max", type=float, default=1.0)
    parser.add_argument("--vy_max", type=float, default=1.0)
    parser.add_argument("--wz_max", type=float, default=1.0)
    parser.add_argument("--robot_root", default="/World/Go2")
    parser.add_argument("--cmd_vel_only", action="store_true", default=False)
    parser.add_argument(
        "--no_sensors", action="store_true", help="Disable sensor setup"
    )
    parser.add_argument(
        "--no_keyboard", action="store_true", help="Disable keyboard control"
    )
    parser.add_argument("--real_time", action="store_true", default=False)
    parser.add_argument("--physics_dt", type=float, default=1 / 200.0)
    parser.add_argument("--render_dt", type=float, default=1 / 60.0)
    args, _ = parser.parse_known_args()

    from isaacsim.core.utils import extensions

    extensions.enable_extension("isaacsim.ros2.bridge")
    extensions.enable_extension("isaacsim.sensors.physics")
    extensions.enable_extension("isaacsim.sensors.rtx")
    simulation_app.update()

    try:
        runner = Go2RosRunner(
            physics_dt=args.physics_dt,
            render_dt=args.render_dt,
            policy_dir=args.policy_dir,
            cmd_vel_topic=args.cmd_vel_topic,
            vx_max=args.vx_max,
            vy_max=args.vy_max,
            wz_max=args.wz_max,
            robot_root=args.robot_root,
            cmd_vel_only=args.cmd_vel_only,
            enable_sensors=not args.no_sensors,
            enable_keyboard=not args.no_keyboard,
        )
        simulation_app.update()
        runner._world.reset()
        simulation_app.update()
        runner.setup()
        simulation_app.update()
        runner.setup_ros()
        simulation_app.update()
        runner.run(real_time=args.real_time)
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
