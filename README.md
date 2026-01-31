# OM1 ROS2 SDK

A ROS 2 package that provides SLAM (Simultaneous Localization and Mapping) capabilities for Unitree Go2, Unitree G1, and Tron robots using an RPLiDAR sensor, the SLAM Toolbox and the Nav2 stack.

## Overview

This package integrates:
- **RPLiDAR** for laser scanning
- **SLAM Toolbox** for mapping and localization
- **Nav2** for autonomous navigation
- **Gazebo Simulation** for supported robot platforms

## Features

- **Real-time SLAM**: Simultaneous localization and mapping using SLAM Toolbox
- **RPLiDAR Integration**: Support for RPLiDAR A1/A2/A3 series sensors
- **Navigation**: Integration with Nav2 for autonomous navigation
- **Multi-Robot Support**: Compatible with Unitree Go2, Unitree G1, and Tron robots
- **Visualization**: Pre-configured RViz setup for monitoring
- **Auto Charging**: AprilTag-based visual docking combined with Nav2 navigation
- **3D SLAM Map**: Builds a 3D SLAM map to help the robot navigate
- **Gazebo Simulation**: Simulator environment for supported platforms

## Supported Robots

- **[Unitree Go2](docs/GO2.md)** - Unitree Go2 with full SLAM and navigation support
- **[Unitree G1](docs/G1.md)** - Unitree G1 with full SLAM and navigation support
- **[Tron](docs/TRON.md)** - LimX Tron with basic object avoidance support

## Prerequisites

- ROS 2 Humble
- Python 3.10+
- RPLiDAR connected via USB (typically `/dev/ttyUSB0`)
- Intel D435i camera

> [!NOTE]
> For getting the best experience, we recommend using [OpenMind Brainpack](https://openmind.org/store) with the robots.

## Quick Start

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenMind/OM1-ros2-sdk.git
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths . --ignore-src -r -y
pip install -e .
```

### 3. Build the workspace

```bash
colcon build
source install/setup.bash
```

### 4. Launch SLAM

For detailed setup instructions for each robot, see the robot-specific guides linked above.

Quick example for Unitree Go2:

```bash
# Launch sensor nodes
ros2 launch go2_sdk sensor_launch.py

# Launch SLAM system
ros2 launch go2_sdk slam_launch.py
```

## Documentation

### Robot-Specific Guides
- **[Unitree Go2 Setup](docs/GO2.md)** - Hardware setup, installation, and usage for Go2
- **[Unitree G1 Setup](docs/G1.md)** - Hardware setup, installation, and usage for G1
- **[Tron Setup](docs/TRON.md)** - Hardware setup, installation, and usage for Tron

### Additional Documentation
- **[REST API Documentation](docs/API.md)** - Complete API reference for remote control
- **[Production Deployment](docs/DEPLOYMENT.md)** - Docker deployment and production setup
- **[Gazebo Simulation](docs/GAZEBO.md)** - Simulation environment setup and usage

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For issues and questions:
- Open an issue on GitHub
- See robot-specific troubleshooting guides in the documentation
