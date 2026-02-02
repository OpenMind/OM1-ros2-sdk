# Unitree Go2 Setup Guide

This guide provides detailed instructions for setting up and using the OM1 ROS2 SDK with the Unitree Go2 robot.

## Hardware Setup

You can place the RPLiDAR at the center of the robot using a 3D-printed mount.

![Front View](./body.png)
![Side View](./head.png)

You can find the 3D model for the mount in the `docs` directory of this repository. We'll shortly be releasing the BOM and details on DIY for it. Stay tuned!

## Installation

### 1. Clone the repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenMind/OM1-ros2-sdk.git
```

### 2. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths . --ignore-src -r -y
```

### 3. Install Python dependencies

```bash
pip install -r requirements.txt
```

### 4. Build the packages

```bash
colcon build --packages-select unitree_api unitree_go go2_sdk
```

### 5. Source the workspace

```bash
source install/setup.bash
```

### 6. Set up RPLiDAR permissions

```bash
# Option 1: Temporary permission (needs to be run each time)
sudo chmod 777 /dev/ttyUSB0

# Option 2: Add user to dialout group (permanent, requires logout/login)
sudo usermod -a -G dialout $USER

# Option 3: Create udev rule (permanent)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Usage

### Launch SLAM System

To start the complete SLAM system:

```bash
ros2 launch go2_sdk slam_launch.py
```

Once you have done the mapping, you can save the map through the RViz2 interface by clicking on the `Save Map` button and the `Serialize Map` button in the `SlamToolboxPlugin` panel.

### Launch Navigation System

To start the navigation system with SLAM:

```bash
ros2 launch go2_sdk nav2_launch.py map_yaml_file:=<path_to_your_map_yaml_file>
```

### Launch the Sensor System

To start the sensor system (RPLiDAR and Unitree Go2 sensors):

```bash
ros2 launch go2_sdk sensor_launch.py
```

### Control the Robot

Once SLAM is running, you can control the robot using:

```bash
# Using keyboard teleop (install first: sudo apt install ros-humble-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish velocity commands directly
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Or use an Xbox controller to control the robot

> [!NOTE]
> For the Xbox controller, the RB button is the enable/disable button. Different Xbox controllers may have different button mappings, so you may need to adjust the launch files accordingly.

### Visualize in RViz

Launch RViz with the provided configuration:

```bash
rviz2 -d config/rviz.rviz
```

## Zenoh ROS2 Bridge

To run the Zenoh bridge for the Unitree Go2, you need to have the Zenoh ROS 2 bridge installed. You can find the installation instructions in the [Zenoh ROS 2 Bridge documentation](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds)

After installing the Zenoh ROS 2 bridge, you can run it with the following command:

```bash
zenoh-bridge-ros2dds -c ./zenoh/zenoh_bridge_config.json5
```

## Development Guidance

This section provides guidance for developers working on the new OM brainpack system.

![brainpack](./backpack.png)

You can connect your Linux machine to one of the Ethernet ports of the OM brainpack.

Open the network settings and find the network interface that the robot is connected to. In IPv4 settings, change the IPv4 mode to manual, set the address to 192.168.123.100, and set the mask to 255.255.255.0. After completion, click apply and wait for the network to reconnect.

Then you can subscribe to the topics published by the Orin AGX and the Unitree Go2 robot.

## Troubleshooting

### RPLiDAR Connection Issues

1. Check USB connection:
```bash
ls -la /dev/ttyUSB*
```

2. Verify permissions:
```bash
sudo chmod 777 /dev/ttyUSB0
```

3. Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
```

### SLAM Performance Issues

- Reduce `scan_buffer_size` if experiencing dropped messages
- Adjust `correlation_search_space_dimension` for better loop closure
- Modify `loop_search_maximum_distance` based on environment size

### Transform Issues

- Ensure all required transforms are being published
- Check TF tree with: `ros2 run tf2_tools view_frames`
- Verify timing with: `ros2 topic echo /tf`

### Robot Control Issues

- Verify cmd_vel messages are being published: `ros2 topic echo /cmd_vel`
- Ensure the robot is in the correct mode for receiving movement commands

### Timestamp Issues

The ROS topic timestamps from the Unitree Go2 are 12 seconds behind the current system time. Please disable your computer's `Automatic Date & Time` setting and manually sync the timestamp using:

```bash
sudo date -s "@unix"
```

You can find the timestamp from the Unitree Go2 by running:

```bash
ros2 topic echo /utlidar/robot_pose --field header.stamp
```

If the Linux machine has an internet connection, you can share the inet connection with the Unitree Go2 robot to automatically sync the time.

```bash
sudo nmcli connection add type ethernet ifname enp112s0 con-name ethernet-shared
sudo nmcli connection modify ethernet-shared ipv4.method shared
sudo nmcli connection modify ethernet-shared ipv4.addresses 192.168.123.1/24
sudo nmcli connection up ethernet-shared
```

and you can verify the shared connection with:

```bash
nmcli connection show ethernet-shared
```
