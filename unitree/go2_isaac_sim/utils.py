# ruff: noqa: E402

import glob
import logging
import math
import os
import re
from typing import Optional, Tuple

import numpy as np
import torch

logger = logging.getLogger(__name__)

# Constants
WAREHOUSE_STAGE_PATH = "/World/Warehouse"
WAREHOUSE_USD_PATH = "/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"

# Go2 Robot prim path
GO2_STAGE_PATH = "/World/envs/env_0/Robot"

# Sensor prim paths on Go2 base
IMU_PRIM = f"{GO2_STAGE_PATH}/base/imu_link"
CAMERA_LINK_PRIM = f"{GO2_STAGE_PATH}/base/camera_link"
REALSENSE_DEPTH_CAMERA_PRIM = f"{CAMERA_LINK_PRIM}/realsense_depth_camera"
REALSENSE_RGB_CAMERA_PRIM = f"{CAMERA_LINK_PRIM}/realsense_rgb_camera"
GO2_RGB_CAMERA_PRIM = f"{CAMERA_LINK_PRIM}/go2_rgb_camera"
L1_LINK_PRIM = f"{GO2_STAGE_PATH}/base/lidar_l1_link"
L1_LIDAR_PRIM = f"{L1_LINK_PRIM}/lidar_l1_rtx"
VELO_BASE_LINK_PRIM = f"{GO2_STAGE_PATH}/base/velodyne_base_link"
VELO_LASER_LINK_PRIM = f"{VELO_BASE_LINK_PRIM}/laser"
VELO_LIDAR_PRIM = f"{VELO_LASER_LINK_PRIM}/velodyne_vlp16_rtx"

# Odom TF handles
odom_graph_path = "/OdomActionGraph"
odom_tf_trans_attr = None
odom_tf_rot_attr = None

odom_pos_attr = None
odom_orient_attr = None
odom_lin_vel_attr = None
odom_ang_vel_attr = None


def find_latest_checkpoint(log_root: str) -> str:
    """Find the latest checkpoint file in the log directory."""
    log_root = os.path.abspath(log_root)
    candidates = glob.glob(os.path.join(log_root, "**", "model_*.pt"), recursive=True)
    if not candidates:
        raise RuntimeError(f"No checkpoints found under: {log_root}")
    best_it, best_path = -1, None
    for p in candidates:
        m = re.search(r"model_(\d+)\.pt$", os.path.basename(p))
        if m and int(m.group(1)) > best_it:
            best_it, best_path = int(m.group(1)), p
    if best_path is None:
        raise RuntimeError(f"No model_<iter>.pt found under: {log_root}")
    return best_path


def set_base_velocity_command(cm, cmd_tensor) -> None:
    """Set the base velocity command on the command manager."""
    if hasattr(cm, "set_command"):
        cm.set_command("base_velocity", cmd_tensor)
        return
    if hasattr(cm, "get_command"):
        cm.get_command("base_velocity")[:] = cmd_tensor
        return
    if hasattr(cm, "get_term"):
        term = cm.get_term("base_velocity")
        for attr in ("command", "_command", "commands", "_commands"):
            if hasattr(term, attr):
                getattr(term, attr)[:] = cmd_tensor
                return
    raise AttributeError("Could not set base_velocity")


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp a value between a lower and upper bound."""
    return max(lo, min(hi, x))


def yaw_to_quat_xyzw(yaw: float):
    """Convert yaw angle to quaternion in xyzw format."""
    half = yaw * 0.5
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def setup_cmd_vel_graph(topic_name: str = "/cmd_vel") -> Tuple[object, object]:
    """Set up the command velocity subscriber graph for ROS2 integration."""
    import omni.graph.core as og
    from isaacsim.core.utils import extensions
    from isaacsim.core.utils.prims import is_prim_path_valid

    extensions.enable_extension("isaacsim.ros2.bridge")

    graph_path = "/CmdVelActionGraph"
    if not is_prim_path_valid(graph_path):
        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("TwistSub", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "TwistSub.inputs:execIn"),
                    ("ROS2Context.outputs:context", "TwistSub.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ROS2Context.inputs:useDomainIDEnvVar", True),
                    ("TwistSub.inputs:topicName", topic_name),
                    ("TwistSub.inputs:queueSize", 10),
                ],
            },
        )
    twist_node_path = graph_path + "/TwistSub"
    return (
        og.Controller.attribute(twist_node_path + ".outputs:linearVelocity"),
        og.Controller.attribute(twist_node_path + ".outputs:angularVelocity"),
    )


def add_warehouse_environment() -> bool:
    """Add the warehouse environment."""
    import carb
    import omni.usd
    from isaacsim.core.utils import nucleus, stage
    from pxr import Gf, UsdGeom

    assets_root_path = nucleus.get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder.")
        return False

    stage.add_reference_to_stage(
        assets_root_path + WAREHOUSE_USD_PATH, WAREHOUSE_STAGE_PATH
    )
    usd_context = omni.usd.get_context()
    usd_stage = usd_context.get_stage()

    warehouse_prim = usd_stage.GetPrimAtPath(WAREHOUSE_STAGE_PATH)
    if not warehouse_prim or not warehouse_prim.IsValid():
        carb.log_error(f"Could not find warehouse prim at {WAREHOUSE_STAGE_PATH}")
        return False

    warehouse_xform = UsdGeom.Xformable(warehouse_prim)
    warehouse_xform.ClearXformOpOrder()
    translate_op = warehouse_xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(0.0, 0.0, -0.01))
    logger.info("Warehouse environment added successfully")
    return True


def make_ground_invisible() -> None:
    """Make Isaac Lab's ground plane invisible but keep it for physics."""
    import omni.usd
    from pxr import UsdGeom

    usd_context = omni.usd.get_context()
    usd_stage = usd_context.get_stage()

    for path in ["/World/ground", "/World/ground/GroundPlane", "/World/GroundPlane"]:
        prim = usd_stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
                logger.info("Made %s invisible", path)
            for child in prim.GetAllChildren():
                child_imageable = UsdGeom.Imageable(child)
                if child_imageable:
                    child_imageable.MakeInvisible()


def modify_env_config_for_warehouse(env_cfg, robot_pos, robot_yaw):
    """Adjust env config to match the warehouse demo setup."""
    if hasattr(env_cfg, "scene") and hasattr(env_cfg.scene, "robot"):
        robot_cfg = env_cfg.scene.robot
        if hasattr(robot_cfg, "init_state"):
            init_state = robot_cfg.init_state
            if hasattr(init_state, "pos"):
                init_state.pos = robot_pos
                logger.info("Robot init pos: %s", robot_pos)
            if hasattr(init_state, "rot"):
                half_yaw = robot_yaw / 2.0
                init_state.rot = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))

    if hasattr(env_cfg, "curriculum"):
        if hasattr(env_cfg.curriculum, "terrain_levels"):
            env_cfg.curriculum.terrain_levels = None
            logger.info("Disabled terrain_levels curriculum")

    if hasattr(env_cfg, "events") and hasattr(env_cfg.events, "push_robot"):
        env_cfg.events.push_robot = None
        logger.info("Disabled push_robot event")

    if hasattr(env_cfg, "episode_length_s"):
        env_cfg.episode_length_s = 10000.0

    return env_cfg


def set_robot_pose(env, pos, yaw) -> bool:
    """Set the root pose for all envs when supported by the articulation."""
    device = env.unwrapped.device
    num_envs = env.unwrapped.num_envs
    pos_tensor = torch.tensor([pos], device=device, dtype=torch.float32).repeat(
        num_envs, 1
    )
    half_yaw = yaw / 2.0
    quat = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))
    quat_tensor = torch.tensor([quat], device=device, dtype=torch.float32).repeat(
        num_envs, 1
    )

    unwrapped = env.unwrapped
    if hasattr(unwrapped, "scene") and hasattr(unwrapped.scene, "articulations"):
        for name, articulation in unwrapped.scene.articulations.items():
            if hasattr(articulation, "write_root_pose_to_sim"):
                pose = torch.cat([pos_tensor, quat_tensor], dim=-1)
                articulation.write_root_pose_to_sim(pose)
                logger.info("Set %s pose via write_root_pose_to_sim", name)
                return True
    return False


def ensure_link_xform(usd_stage, path: str, translation=None, rpy_rad=None):
    """Ensure a link Xform exists with the specified translation and rotation."""
    from pxr import Gf, UsdGeom

    prim = usd_stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        prim = usd_stage.DefinePrim(path, "Xform")
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    if translation is not None:
        xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    if rpy_rad is not None:
        roll, pitch, yaw_ = [math.degrees(v) for v in rpy_rad]
        xform.AddRotateXYZOp().Set(Gf.Vec3f(roll, pitch, yaw_))
    return prim


def setup_sensors_delayed(simulation_app, render_hz: Optional[float] = None) -> dict:
    """Setup sensors after simulation is fully running."""
    import omni.kit.commands
    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.core.utils.prims import is_prim_path_valid
    from isaacsim.sensors.camera import Camera
    from isaacsim.sensors.physics import IMUSensor
    from pxr import Gf

    usd_context = omni.usd.get_context()
    usd_stage = usd_context.get_stage()

    sensors = {"realsense_depth_camera": None, "realsense_rgb_camera": None, "go2_rgb_camera": None, "imu": None}

    # --- Cameras ---
    try:
        # Camera link
        ensure_link_xform(usd_stage, CAMERA_LINK_PRIM,
                          translation=(0.3, 0.0, 0.35),
                          rpy_rad=(math.radians(90.0), math.radians(0.0), math.radians(-90.0)))

        realsense_depth_camera = Camera(
            prim_path=REALSENSE_DEPTH_CAMERA_PRIM,
            name="realsense_depth_camera",
            resolution=(480, 270),
        )
        realsense_depth_camera.initialize()

        realsense_depth_cam_prim = usd_stage.GetPrimAtPath(REALSENSE_DEPTH_CAMERA_PRIM)
        if realsense_depth_cam_prim and realsense_depth_cam_prim.IsValid():
            from pxr import UsdGeom, Gf
            xformable = UsdGeom.Xformable(realsense_depth_cam_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 0.0))  # 25° downward tilt (X-axis)
            logger.info(f"[Sensors] Set realsense_depth_camera 25° downward tilt, 5cm higher")

        realsense_depth_camera.set_clipping_range(near_distance=0.1, far_distance=100.0)
        realsense_depth_camera.add_distance_to_image_plane_to_frame()

        sensors["realsense_depth_camera"] = realsense_depth_camera
        logger.info(f"[Sensors] RealSense depth camera initialized with depth enabled")

        realsense_rgb_camera = Camera(
            prim_path=REALSENSE_RGB_CAMERA_PRIM,
            name="realsense_rgb_camera",
            resolution=(640, 480),
        )
        realsense_rgb_camera.initialize()

        realsense_rgb_cam_prim = usd_stage.GetPrimAtPath(REALSENSE_RGB_CAMERA_PRIM)
        if realsense_rgb_cam_prim and realsense_rgb_cam_prim.IsValid():
            from pxr import UsdGeom, Gf
            xformable = UsdGeom.Xformable(realsense_rgb_cam_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(-25.0, 0.0, 0.0))  # 25° downward tilt (X-axis)
            logger.info(f"[Sensors] Set realsense_rgb_camera 25° downward tilt, 5cm higher")

        realsense_rgb_camera.set_clipping_range(near_distance=0.1, far_distance=100.0)
        sensors["realsense_rgb_camera"] = realsense_rgb_camera
        logger.info(f"[Sensors] RealSense RGB camera initialized")

        go2_rgb_camera = Camera(
            prim_path=GO2_RGB_CAMERA_PRIM,
            name="go2_rgb_camera",
            resolution=(640, 480),
        )
        go2_rgb_camera.initialize()

        go2_rgb_cam_prim = usd_stage.GetPrimAtPath(GO2_RGB_CAMERA_PRIM)
        if go2_rgb_cam_prim and go2_rgb_cam_prim.IsValid():
            from pxr import UsdGeom, Gf
            xformable = UsdGeom.Xformable(go2_rgb_cam_prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
            xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 25.0, 0.0))  # Counteract camera_link's -25° tilt
            logger.info(f"[Sensors] Set go2_rgb_camera to face forward (counteracting camera_link tilt)")

        go2_rgb_camera.set_clipping_range(near_distance=0.1, far_distance=100.0)
        sensors["go2_rgb_camera"] = go2_rgb_camera
        logger.info(f"[Sensors] Go2 RGB camera initialized")
    except Exception as e:
        logger.info(f"[WARN] Camera setup failed: {e}")
        import traceback
        traceback.print_exc()

    # --- IMU ---
    try:
        imu_sensor = IMUSensor(
            prim_path=IMU_PRIM, name="imu_sensor", frequency=50,
            translation=np.array([0.0, 0.0, 0.0]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        imu_sensor.initialize()
        sensors["imu"] = imu_sensor
        logger.info(f"[Sensors] IMU initialized")
    except Exception as e:
        logger.info(f"[WARN] IMU setup failed: {e}")

    # --- LiDARs ---
    try:
        ensure_link_xform(usd_stage, L1_LINK_PRIM, translation=(0.15, 0.0, 0.15), rpy_rad=(0.0, 0.0, 0.0))
        result = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="lidar_l1_rtx",
            parent=L1_LINK_PRIM,
            config="Example_Rotary",
            translation=(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1, 0, 0, 0)
        )
        if result and len(result) > 1 and result[1]:
            lidar_prim = result[1]
            lidar_path = lidar_prim.GetPath().pathString
            logger.info(f"[Sensors] L1 LiDAR created at: {lidar_path}")
            l1_rp = rep.create.render_product(lidar_path, resolution=(1, 1), name="l1_lidar_rp")
            pc_writer = rep.writers.get("RtxLidarROS2PublishPointCloud")
            pc_writer.initialize(frameId="lidar_l1_link", nodeNamespace="", topicName="/unitree_lidar", queueSize=10)
            pc_writer.attach([l1_rp])
            logger.info(f"[Sensors] L1 LiDAR -> /unitree_lidar")
        else:
            logger.info(f"[WARN] L1 LiDAR creation returned: {result}")
    except Exception as e:
        logger.info(f"[WARN] L1 LiDAR setup failed: {e}")
        import traceback
        traceback.print_exc()

    try:
        ensure_link_xform(usd_stage, VELO_BASE_LINK_PRIM, translation=(0.1, 0.0, 0.2), rpy_rad=(0.0, 0.0, 0.0))
        ensure_link_xform(usd_stage, VELO_LASER_LINK_PRIM, translation=(0.0, 0.0, 0.0377), rpy_rad=(0.0, 0.0, 0.0))
        result = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="rplidar",
            parent=VELO_LASER_LINK_PRIM,
            config="Slamtec_RPLIDAR_S2E",
            translation=(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1, 0, 0, 0)
        )
        if result and len(result) > 1 and result[1]:
            lidar_prim = result[1]
            lidar_path = lidar_prim.GetPath().pathString
            logger.info(f"[Sensors] 2D LiDAR created at: {lidar_path}")
            velo_rp = rep.create.render_product(lidar_path, resolution=(1, 1), name="velo_lidar_rp")
            scan_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
            scan_writer.initialize(frameId="laser", nodeNamespace="", topicName="/scan", queueSize=10)
            scan_writer.attach([velo_rp])
            logger.info(f"[Sensors] 2D LiDAR -> /scan")
        else:
            logger.info(f"[WARN] 2D LiDAR creation returned: {result}")
    except Exception as e:
        logger.info(f"[WARN] 2D LiDAR setup failed: {e}")
        import traceback
        traceback.print_exc()

    simulation_app.update()
    return sensors


def setup_static_tfs(simulation_app) -> None:
    """Publish static TFs for sensor frames to complete the TF tree."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid

    graph_path = "/StaticTFGraph"
    if is_prim_path_valid(graph_path):
        logger.info("[ROS2] Static TF graph already exists")
        return

    # Define all the static transforms for Go2
    # Format: (parent, child, translation, rotation_xyzw)
    static_transforms = [
        ("base_link", "base", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ("base", "lidar_l1_link", [0.15, 0.0, 0.15], [0.0, 0.0, 0.0, 1.0]),
        ("base", "velodyne_base_link", [0.1, 0.0, 0.2], [0.0, 0.0, 0.0, 1.0]),
        ("velodyne_base_link", "laser", [0.0, 0.0, 0.0377], [0.0, 0.0, 0.0, 1.0]),
        ("base", "imu_link", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ("base", "camera_link", [0.3, 0.0, 0.1], [0.5, -0.5, -0.5, 0.5]),
        ("camera_link", "realsense_depth_camera", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ("camera_link", "realsense_rgb_camera", [0.0, 0.05, 0.0], [0.0, 0.0, 0.0, 1.0]),
        ("base_link", "realsense_depth_camera", [0.3, 0.0, 0.1], [0.5, -0.5, -0.5, 0.5]),
        ("map", "odom", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
    ]

    create_nodes = [
        ("OnTick", "omni.graph.action.OnTick"),
        ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
    ]

    for i, _ in enumerate(static_transforms):
        create_nodes.append(
            (f"TF{i}", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
        )

    connections = []
    for i, _ in enumerate(static_transforms):
        connections.append(("OnTick.outputs:tick", f"TF{i}.inputs:execIn"))
        connections.append(("Clock.outputs:simulationTime", f"TF{i}.inputs:timeStamp"))
        connections.append(("Ctx.outputs:context", f"TF{i}.inputs:context"))

    set_values = [("Ctx.inputs:useDomainIDEnvVar", True)]

    for i, (parent, child, trans, rot) in enumerate(static_transforms):
        set_values.extend(
            [
                (f"TF{i}.inputs:parentFrameId", parent),
                (f"TF{i}.inputs:childFrameId", child),
                (f"TF{i}.inputs:topicName", "/tf_static"),
                (f"TF{i}.inputs:translation", trans),
                (f"TF{i}.inputs:rotation", rot),
                (f"TF{i}.inputs:staticPublisher", True),
                (f"TF{i}.inputs:queueSize", 10),
            ]
        )

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connections,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    logger.info(f"[ROS2] Static TFs published for {len(static_transforms)} transforms (staticPublisher=True)")
    simulation_app.update()


def setup_odom_publisher(simulation_app) -> None:
    """Publish nav_msgs/Odometry on /odom topic."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid

    global odom_pos_attr, odom_orient_attr, odom_lin_vel_attr, odom_ang_vel_attr

    graph_path = "/OdomPublisherGraph"
    if is_prim_path_valid(graph_path):
        logger.info("[ROS2] Odom publisher graph already exists")
        return

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("OdomPub", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "OdomPub.inputs:execIn"),
                ("Clock.outputs:simulationTime", "OdomPub.inputs:timeStamp"),
                ("Ctx.outputs:context", "OdomPub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Ctx.inputs:useDomainIDEnvVar", True),
                ("OdomPub.inputs:topicName", "/odom"),
                ("OdomPub.inputs:odomFrameId", "odom"),
                ("OdomPub.inputs:chassisFrameId", "base_link"),
                ("OdomPub.inputs:queueSize", 10),
                # Initialize with zeros - will be updated each frame
                ("OdomPub.inputs:position", [0.0, 0.0, 0.0]),
                ("OdomPub.inputs:orientation", [0.0, 0.0, 0.0, 1.0]),  # xyzw
                ("OdomPub.inputs:linearVelocity", [0.0, 0.0, 0.0]),
                ("OdomPub.inputs:angularVelocity", [0.0, 0.0, 0.0]),
            ],
        },
    )

    # Get attribute handles for updating each frame
    odom_pos_attr = og.Controller.attribute(graph_path + "/OdomPub.inputs:position")
    odom_orient_attr = og.Controller.attribute(
        graph_path + "/OdomPub.inputs:orientation"
    )
    odom_lin_vel_attr = og.Controller.attribute(
        graph_path + "/OdomPub.inputs:linearVelocity"
    )
    odom_ang_vel_attr = og.Controller.attribute(
        graph_path + "/OdomPub.inputs:angularVelocity"
    )

    logger.info("[ROS2] Odometry publisher -> /odom")
    simulation_app.update()


def update_odom(pos, quat_xyzw, lin_vel, ang_vel) -> None:
    """Update the odometry message each frame."""
    global odom_pos_attr, odom_orient_attr, odom_lin_vel_attr, odom_ang_vel_attr

    if odom_pos_attr is not None:
        odom_pos_attr.set([float(pos[0]), float(pos[1]), float(pos[2])])
    if odom_orient_attr is not None:
        odom_orient_attr.set(
            [
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
                float(quat_xyzw[3]),
            ]
        )
    if odom_lin_vel_attr is not None:
        odom_lin_vel_attr.set([float(lin_vel[0]), float(lin_vel[1]), float(lin_vel[2])])
    if odom_ang_vel_attr is not None:
        odom_ang_vel_attr.set([float(ang_vel[0]), float(ang_vel[1]), float(ang_vel[2])])


def setup_color_camera_publishers(sensors, simulation_app) -> None:
    """Set up ROS2 publishers for color camera images."""
    import omni.replicator.core as rep
    import omni.syntheticdata as syn_data
    import omni.syntheticdata._syntheticdata as sd

    if sensors.get("realsense_depth_camera"):
        cam = sensors["realsense_depth_camera"]
        rp = cam.get_render_product_path()
        if rp:
            try:
                # Color image on RealSense topic
                rv = syn_data.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
                w = rep.writers.get(rv + "ROS2PublishImage")
                w.initialize(
                    frameId="realsense_depth_camera",
                    nodeNamespace="",
                    queueSize=10,
                    topicName="/camera/realsense2_camera_node/color/image_isaac_sim_raw"
                )
                w.attach([rp])
                logger.info("[ROS2] Color camera -> /camera/realsense2_camera_node/color/image_isaac_sim_raw")

            except Exception as e:
                logger.info(f"[WARN] Color camera publisher setup failed: {e}")

    # Go2 RGB Camera
    if sensors.get("go2_rgb_camera"):
        cam = sensors["go2_rgb_camera"]
        rp = cam.get_render_product_path()
        if rp:
            try:
                # RGB image for Go2 camera
                rv = syn_data.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
                w = rep.writers.get(rv + "ROS2PublishImage")
                w.initialize(
                    frameId="go2_rgb_camera",
                    nodeNamespace="",
                    queueSize=10,
                    topicName="camera/go2/image_raw"
                )
                w.attach([rp])
                logger.info("[ROS2] Go2 RGB camera -> camera/go2/image_raw")

            except Exception as e:
                logger.info(f"[WARN] Go2 RGB camera publisher setup failed: {e}")


def setup_color_camerainfo_graph(
    simulation_app,
    topic="/camera/realsense2_camera_node/color/camera_info",
    frame_id="realsense_depth_camera",
    width=480,
    height=270,
    fx=320.0,
    fy=320.0,
    cx=None,
    cy=None,
) -> bool:
    """Publish CameraInfo for color camera."""
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid

    graph_path = "/ColorCameraInfoGraph"
    if is_prim_path_valid(graph_path):
        logger.info("[ROS2] Color CameraInfo graph already exists")
        return True

    if cx is None:
        cx = width / 2.0
    if cy is None:
        cy = height / 2.0

    K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("Pub", "isaacsim.ros2.bridge.ROS2PublishCameraInfo"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "Pub.inputs:execIn"),
                ("Clock.outputs:simulationTime", "Pub.inputs:timeStamp"),
                ("Ctx.outputs:context", "Pub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Ctx.inputs:useDomainIDEnvVar", True),
                ("Pub.inputs:topicName", topic),
                ("Pub.inputs:frameId", frame_id),
                ("Pub.inputs:queueSize", 10),
                ("Pub.inputs:width", width),
                ("Pub.inputs:height", height),
                ("Pub.inputs:k", K),
                ("Pub.inputs:r", R),
                ("Pub.inputs:p", P),
                ("Pub.inputs:physicalDistortionModel", "plumb_bob"),
                ("Pub.inputs:physicalDistortionCoefficients", [0.0, 0.0, 0.0, 0.0, 0.0]),
            ],
        },
    )

    logger.info(f"[ROS2] Color CameraInfo -> {topic}")
    simulation_app.update()
    return True


def setup_joint_states_publisher(simulation_app) -> None:
    """Publish sensor_msgs/JointState on /joint_states topic."""
    import omni.graph.core as og
    from isaacsim.core.nodes.scripts.utils import set_target_prims
    from isaacsim.core.utils.prims import is_prim_path_valid

    graph_path = "/JointStatesGraph"
    if is_prim_path_valid(graph_path):
        logger.info("[ROS2] Joint states graph already exists")
        return

    ROBOT_ARTICULATION_PATH = f"{GO2_STAGE_PATH}/base"

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("JointStatePub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "JointStatePub.inputs:execIn"),
                ("Clock.outputs:simulationTime", "JointStatePub.inputs:timeStamp"),
                ("Ctx.outputs:context", "JointStatePub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Ctx.inputs:useDomainIDEnvVar", True),
                ("JointStatePub.inputs:topicName", "/joint_states"),
                ("JointStatePub.inputs:queueSize", 10),
            ],
        },
    )

    set_target_prims(
        primPath=graph_path + "/JointStatePub",
        inputName="inputs:targetPrim",
        targetPrimPaths=[ROBOT_ARTICULATION_PATH],
    )

    logger.info(f"[ROS2] Joint states publisher -> /joint_states (articulation: {ROBOT_ARTICULATION_PATH})")
    simulation_app.update()


def setup_ros_publishers(sensors, simulation_app) -> None:
    """Setup ROS2 publishers for sensors."""
    import omni.graph.core as og
    import omni.replicator.core as rep
    import omni.syntheticdata as syn_data
    import omni.syntheticdata._syntheticdata as sd
    from isaacsim.core.utils.prims import is_prim_path_valid

    # Clock publisher
    graph_path = "/ClockGraph"
    if not is_prim_path_valid(graph_path):
        og.Controller.edit(
            {
                "graph_path": graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Pub", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "Pub.inputs:execIn"),
                    ("Clock.outputs:simulationTime", "Pub.inputs:timeStamp"),
                ],
            },
        )
    logger.info("[ROS2] Clock publisher -> /clock")

    # IMU publisher
    if not is_prim_path_valid("/ImuGraph"):
        og.Controller.edit(
            {
                "graph_path": "/ImuGraph",
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                    ("Read", "isaacsim.sensors.physics.IsaacReadIMU"),
                    ("Pub", "isaacsim.ros2.bridge.ROS2PublishImu"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "Read.inputs:execIn"),
                    ("Read.outputs:execOut", "Pub.inputs:execIn"),
                    ("Ctx.outputs:context", "Pub.inputs:context"),
                    ("Clock.outputs:simulationTime", "Pub.inputs:timeStamp"),
                    ("Read.outputs:angVel", "Pub.inputs:angularVelocity"),
                    ("Read.outputs:linAcc", "Pub.inputs:linearAcceleration"),
                    ("Read.outputs:orientation", "Pub.inputs:orientation"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Ctx.inputs:useDomainIDEnvVar", True),
                    ("Read.inputs:imuPrim", IMU_PRIM),
                    ("Read.inputs:readGravity", True),
                    ("Pub.inputs:frameId", "imu_link"),
                    ("Pub.inputs:topicName", "imu/data"),
                    ("Pub.inputs:queueSize", 10),
                ],
            },
        )
    logger.info("[ROS2] IMU publisher -> imu/data")

    # Camera publishers with CameraInfo
    if sensors.get("realsense_depth_camera"):
        cam = sensors["realsense_depth_camera"]
        rp = cam.get_render_product_path()
        if rp:
            try:
                # Depth Image
                rv = syn_data.SyntheticData.convert_sensor_type_to_rendervar(
                    sd.SensorType.DistanceToImagePlane.name
                )
                w_rs_depth = rep.writers.get(rv + "ROS2PublishImage")
                w_rs_depth.initialize(
                    frameId="realsense_depth_camera",
                    nodeNamespace="",
                    queueSize=10,
                    topicName="/camera/realsense2_camera_node/depth/image_rect_isaac_sim_raw",
                )
                w_rs_depth.attach([rp])
                logger.info("[ROS2] Depth camera -> /camera/realsense2_camera_node/depth/image_rect_isaac_sim_raw")

                # For easier RViz viewing
                try:
                    depth_colorized = rep.writers.get("ROS2PublishNormalized" + "DepthImage")
                    depth_colorized.initialize(
                        frameId="realsense_depth_camera",
                        nodeNamespace="",
                        queueSize=10,
                        topicName="camera/depth/image_colorized"
                    )
                    depth_colorized.attach([rp])
                    logger.info("[ROS2] Depth colorized -> camera/depth/image_colorized")
                except Exception as de:
                    logger.info(f"[INFO] Normalized depth writer not available: {de}")

            except Exception as e:
                logger.info(f"[WARN] Camera publisher setup failed: {e}")
                import traceback
                traceback.print_exc()

    # Setup static TFs for sensor frames
    setup_static_tfs(simulation_app)

    # Odom TF publisher (dynamic - updated each frame)
    global odom_tf_trans_attr, odom_tf_rot_attr
    if not is_prim_path_valid(odom_graph_path):
        og.Controller.edit(
            {
                "graph_path": odom_graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                    ("TF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "TF.inputs:execIn"),
                    ("Clock.outputs:simulationTime", "TF.inputs:timeStamp"),
                    ("Ctx.outputs:context", "TF.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Ctx.inputs:useDomainIDEnvVar", True),
                    ("TF.inputs:parentFrameId", "odom"),
                    ("TF.inputs:childFrameId", "base_link"),
                    ("TF.inputs:topicName", "/tf"),
                ],
            },
        )
    odom_tf_trans_attr = og.Controller.attribute(
        odom_graph_path + "/TF.inputs:translation"
    )
    odom_tf_rot_attr = og.Controller.attribute(odom_graph_path + "/TF.inputs:rotation")
    logger.info("[ROS2] Odom TF -> /tf (odom->base_link)")

    simulation_app.update()


def setup_depth_camerainfo_graph(
    simulation_app,
    topic="/camera/realsense2_camera_node/depth/camera_info",
    frame_id="realsense_depth_camera",
    width=480,
    height=270,
    fx=320.0,
    fy=320.0,
    cx=None,
    cy=None,
) -> bool:
    """
    Publish depth CameraInfo
    """
    import omni.graph.core as og
    from isaacsim.core.utils.prims import is_prim_path_valid

    graph_path = "/DepthCameraInfoGraph"
    if is_prim_path_valid(graph_path):
        logger.info("[ROS2] Depth CameraInfo graph already exists")
        return True

    if cx is None:
        cx = width / 2.0
    if cy is None:
        cy = height / 2.0

    # K matrix (3x3 intrinsic matrix, row-major)
    K = [fx, 0.0, cx,
         0.0, fy, cy,
         0.0, 0.0, 1.0]

    # R matrix (3x3 rectification matrix, identity for monocular)
    R = [1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0]

    # P matrix (3x4 projection matrix)
    P = [fx, 0.0, cx, 0.0,
         0.0, fy, cy, 0.0,
         0.0, 0.0, 1.0, 0.0]

    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("Clock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                ("Pub", "isaacsim.ros2.bridge.ROS2PublishCameraInfo"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick", "Pub.inputs:execIn"),
                ("Clock.outputs:simulationTime", "Pub.inputs:timeStamp"),
                ("Ctx.outputs:context", "Pub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Ctx.inputs:useDomainIDEnvVar", True),
                ("Pub.inputs:topicName", topic),
                ("Pub.inputs:frameId", frame_id),
                ("Pub.inputs:queueSize", 10),
                ("Pub.inputs:width", width),
                ("Pub.inputs:height", height),
                ("Pub.inputs:k", K),
                ("Pub.inputs:r", R),
                ("Pub.inputs:p", P),
                ("Pub.inputs:physicalDistortionModel", "plumb_bob"),
                ("Pub.inputs:physicalDistortionCoefficients", [0.0, 0.0, 0.0, 0.0, 0.0]),
            ],
        },
    )

    logger.info(f"[ROS2] Depth CameraInfo -> {topic} (width={width}, height={height}, fx={fx}, fy={fy})")
    simulation_app.update()
    return True


def update_odom_tf(pos, quat_xyzw) -> None:
    """Update the odom -> base_link transform each frame."""
    if odom_tf_trans_attr is not None and odom_tf_rot_attr is not None:
        odom_tf_trans_attr.set([float(pos[0]), float(pos[1]), float(pos[2])])
        odom_tf_rot_attr.set(
            [
                float(quat_xyzw[0]),
                float(quat_xyzw[1]),
                float(quat_xyzw[2]),
                float(quat_xyzw[3]),
            ]
        )


def find_robot_articulation_path():
    """Find the actual robot articulation path in the stage."""
    import omni.usd
    from pxr import UsdPhysics

    usd_context = omni.usd.get_context()
    usd_stage = usd_context.get_stage()

    logger.debug("Searching for articulation roots in stage...")

    # Find all ArticulationRootAPI prims
    articulations = []
    for prim in usd_stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            articulations.append(prim.GetPath().pathString)
            logger.debug("  [ArticulationRoot] %s", prim.GetPath().pathString)

    # Check for common robot paths
    common_paths = [
        "/World/envs/env_0/Robot",
        "/World/envs/env_0/Robot/base",
        "/World/Go2",
        "/World/go2",
        "/World/robot",
    ]

    logger.debug("Checking common paths:")
    for path in common_paths:
        prim = usd_stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            has_arctic = prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            logger.debug("  %s: exists=True, has_articulation_api=%s", path, has_arctic)
        else:
            logger.debug("  %s: exists=False", path)

    # List children of /World/envs/env_0/Robot if it exists
    robot_prim = usd_stage.GetPrimAtPath("/World/envs/env_0/Robot")
    if robot_prim and robot_prim.IsValid():
        logger.debug("Children of /World/envs/env_0/Robot:")
        for child in robot_prim.GetChildren():
            has_arctic = child.HasAPI(UsdPhysics.ArticulationRootAPI)
            logger.debug(
                "  %s (articulation=%s)", child.GetPath().pathString, has_arctic
            )

    # List direct children of /World/envs/env_0
    env_prim = usd_stage.GetPrimAtPath("/World/envs/env_0")
    if env_prim and env_prim.IsValid():
        logger.debug("Children of /World/envs/env_0:")
        for child in env_prim.GetChildren():
            has_arctic = child.HasAPI(UsdPhysics.ArticulationRootAPI)
            logger.debug(
                "  %s (articulation=%s)", child.GetPath().pathString, has_arctic
            )

    return articulations
