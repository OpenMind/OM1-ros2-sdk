# Gazebo Simulation Guide

This guide covers setting up and using the Gazebo simulator with the OM1 ROS2 SDK.

## Prerequisites

To set up the **Gazebo Simulator**, please install **ROS2 Humble** first.

> [!NOTE]
> If you are using **ROS2 Jazzy**, you might need to manually modify the configuration file for **SLAM** and **NAV2** to ensure compatibility.

## Installation

Once **ROS2 Humble** is installed, install all necessary dependencies.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

## Configuration

Use the following **CycloneDDS** configuration. It uses `lo` as the network interface.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
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

## Running the Simulator

### Start the Gazebo Simulator (Go2)

```bash
ros2 launch go2_gazebo_sim go2_launch.py
```

### Start the OM1 Sensor

To generate the `OM Path`:

```bash
ros2 launch go2_sdk sensor_launch.py use_sim:=true
```

### Start Auto SLAM

To automatically generate the map:

```bash
ros2 launch go2_sdk slam_auto_launch.py use_sim:=true
```

## Supported Platforms

Currently, Gazebo simulation is available for:
- Unitree Go2

Support for G1 and Tron is planned for future releases.
