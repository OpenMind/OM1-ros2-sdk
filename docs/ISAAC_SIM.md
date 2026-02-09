# Isaac Sim Simulation Guide

This guide covers setting up and using the NVIDIA Isaac Sim simulator with the OM1 ROS2 SDK for the Unitree Go2 robot.

## Overview

The Isaac Sim integration provides a high-fidelity physics simulation environment for the Unitree Go2 robot with:

- **Realistic Physics**: GPU-accelerated PhysX simulation for accurate robot dynamics
- **ROS2 Integration**: Full ROS2 bridge support for seamless integration with the OM1 SDK
- **Sensor Simulation**: Support for LiDAR (Velodyne VLP-16, Unitree 4D LiDAR), cameras, and IMU
- **Reinforcement Learning**: Pre-trained locomotion policies for velocity tracking
- **Real-time Control**: Keyboard and `/cmd_vel` topic control support

## Prerequisites

### System Requirements

- **OS**: Ubuntu 20.04 or 22.04 (recommended)
- **GPU**: NVIDIA RTX GPU with at least 8GB VRAM
- **CUDA**: CUDA 11.8 or 12.8
- **Python**: 3.10 or 3.11
- **ROS2**: Humble (recommended)

> [!NOTE]
> Isaac Sim requires a compatible NVIDIA GPU with RTX capabilities. Check the [official requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html) for more details.

## Installation

### 1. Set up Python Virtual Environment

Create a dedicated Python environment for Isaac Sim:

```bash
# Install uv if not already installed
pip install uv

# Create virtual environment with Python 3.11
uv venv --python 3.11 --seed env_isaacsim

# Activate the environment
source env_isaacsim/bin/activate
```

### 2. Install Isaac Sim

Install Isaac Sim 5.1.0 with all required extensions:

```bash
# Install Isaac Sim with all extensions and extscache
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
```

> [!WARNING]
> The installation may take 10-20 minutes and requires approximately 10GB of disk space.

### 3. Install PyTorch

Install a CUDA-enabled PyTorch build matching your system:

```bash
# For CUDA 12.8 (recommended for RTX 40 series)
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

# For CUDA 11.8 (for older GPUs)
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu118
```

### 4. Verify Installation

Test that Isaac Sim is correctly installed:

```bash
isaacsim
```

This should launch the Isaac Sim GUI. Close it once you verify it works.

## Configuration

### ROS2 Environment Setup

Before running the simulation, configure your ROS2 environment:

```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:PATH_TO_VENV/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
```

> [!IMPORTANT]
> Replace `PATH_TO_VENV` with the actual absolute path to your virtual environment directory.

To make this permanent, add these lines to your `~/.bashrc`:

```bash
echo 'export ROS_DISTRO=humble' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$HOME/env_isaacsim/lib/python3.11/site-packages/isaacsim/exts/isaacsim.ros2.bridge/humble/lib" >> ~/.bashrc
source ~/.bashrc
```

### CycloneDDS Configuration

For optimal ROS2 communication, use the CycloneDDS configuration:

```bash
export CYCLONEDDS_URI='
<CycloneDDS>
    <Domain>
        <General>
            <Interfaces>
                <NetworkInterface address="127.0.0.1" priority="default" multicast="default" />
            </Interfaces>
        </General>
        <Discovery>
            <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
        </Discovery>
    </Domain>
</CycloneDDS>'
```

Or use the provided configuration file:

```bash
export CYCLONEDDS_URI=file://$PWD/cyclonedds/cyclonedds.xml
```

## Running the Simulation

### Basic Usage

Navigate to the Isaac Sim directory and run the simulation:

```bash
cd unitree/go2_isaac_sim
source env_isaacsim/bin/activate

# Run with default policy
python3 run.py

# Run with custom policy directory
python3 run.py --policy_dir /path/to/your/policy
```

### Command Line Arguments

The `run.py` script supports the following arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `--policy_dir` | `checkpoints/50000` | Path to trained policy checkpoint |
| `--cmd_vel_topic` | `/cmd_vel` | ROS2 topic for velocity commands |
| `--vx_max` | `1.0` | Maximum linear velocity in x-direction (m/s) |
| `--vy_max` | `1.0` | Maximum linear velocity in y-direction (m/s) |
| `--wz_max` | `1.0` | Maximum angular velocity (rad/s) |
| `--robot_root` | `/World/Go2` | USD prim path for the robot |
| `--cmd_vel_only` | `False` | Use only `/cmd_vel` (disable keyboard) |
| `--no_sensors` | `False` | Disable sensor publishers |
| `--no_keyboard` | `False` | Disable keyboard control |
| `--real_time` | `False` | Enable real-time simulation mode |
| `--physics_dt` | `0.005` | Physics timestep (seconds) |
| `--render_dt` | `0.0167` | Render timestep (seconds) |

### Example Usage Scenarios

#### 1. Run with Sensors Enabled (Default)

```bash
python3 run.py
```

This enables:
- IMU data publishing
- Camera image streaming (depth and color)
- LiDAR point cloud publishing
- Joint state publishing
- Odometry publishing

#### 2. Run without Sensors (Faster Simulation)

```bash
python3 run.py --no_sensors
```

#### 3. Run with Custom Velocity Limits

```bash
python3 run.py --vx_max 2.0 --vy_max 0.5 --wz_max 1.5
```

#### 4. Real-time Simulation Mode

```bash
python3 run.py --real_time
```

## Control Methods

### Keyboard Control

When keyboard control is enabled (default), use the following keys:

| Key | Action |
|-----|--------|
| `↑` or `Numpad 8` | Move forward |
| `↓` or `Numpad 2` | Move backward |
| `←` or `Numpad 4` | Strafe left |
| `→` or `Numpad 6` | Strafe right |
| `N` or `Numpad 7` | Rotate left |
| `M` or `Numpad 9` | Rotate right |

> [!NOTE]
> Commands are additive - you can press multiple keys simultaneously for combined motion.

### ROS2 `/cmd_vel` Topic

Control the robot through ROS2:

```bash
# Install teleop keyboard if needed
sudo apt install ros-humble-teleop-twist-keyboard

# Run teleop in a new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly
ros2 topic pub /cmd_vel geometry_msgs/Twist '{
  linear: {x: 0.5, y: 0.0, z: 0.0},
  angular: {x: 0.0, y: 0.0, z: 0.3}
}'
```
