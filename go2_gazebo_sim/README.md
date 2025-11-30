# Unitree Go2 Gazebo Simulation (go2_gazebo_sim)

This repository contains the ROS 2 Gazebo simulation packages for the Unitree Go2 quadruped robot. It provides a complete simulation environment with Gazebo, including mapping and navigation capabilities with support for tight-space navigation testing.

## Features
- **Gazebo Simulation**: Realistic physics simulation of the Unitree Go2.
- **Navigation Stack (Nav2)**: Fully configured navigation stack for autonomous movement.
- **SLAM**: Mapping capabilities using `slam_toolbox`.
- **LiDAR Support**: Simulation of Velodyne VLP-16 and Unitree 4D LiDAR.
- **Mini Maze World**: Test environment with narrow corridors (0.85-0.9m), ramps, and obstacles for tight-space navigation testing.

## Installation

1.  Install dependencies:
    ```bash
    rosdep install --from-paths . --ignore-src -r -y
    ```

2.  Build the workspace:
    ```bash
    colcon build --symlink-install
    ```

3.  Source the workspace:
    ```bash
    source install/setup.bash
    ```

4.  Setup CycloneDDS
    The network interface is your WiFi adapter. Replace `wlxbc071d7223b6` with your own network interface name found via `ifconfig` or `ip a`.
    ```bash
    <CycloneDDS>
        <Domain>
            <General>
            <Interfaces>
                <NetworkInterface name="wlxbc071d7223b6" priority="default" multicast="default" />
            </Interfaces>
            </General>
        </Domain>
    </CycloneDDS>
    ```

## Usage

### 1. Launch SLAM (Mapping)
To launch the simulation and start mapping the environment:
```bash
ros2 launch go2_gazebo_sim go2_slam_launch.py
```

**Default world**: `maze_world.sdf` (mini maze with narrow corridors and slopes)

To use a different world:
```bash
ros2 launch go2_gazebo_sim go2_slam_launch.py world:=default.sdf
```

#### Control the Robot with Teleop
In a new terminal, run the teleop keyboard node to drive the robot around:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard controls displayed in the terminal to move the robot:
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `u/o/m/.` - Move diagonally

Drive the robot around to cover the entire area and build a complete map.

### 2. Save the Map
Once you are satisfied with the map, save it using the map saver CLI:
```bash
cd ~
ros2 run nav2_map_server map_saver_cli -f my_map
```
This will create `my_map.yaml` and `my_map.pgm` in your home directory.

### 3. Launch Navigation
To launch the simulation with the navigation stack using your saved map:
```bash
ros2 launch go2_gazebo_sim go2_nav_launch.py map:=${HOME}/my_map.yaml
```

To use a different world:
```bash
ros2 launch go2_gazebo_sim go2_nav_launch.py world:=default.sdf map:=${HOME}/my_map.yaml
```

You can now set 2D Pose Estimates and Navigation Goals in RViz.

## Packages
- `go2_gazebo_sim`: Main Gazebo simulation launch files and configurations.
- `go2_description`: URDF, mesh files, and world definitions for the robot.
- `champ`: Quadruped controller framework.

## World Files
- **maze_world.sdf** (default): Mini maze with narrow passages (0.85-0.9m wide), upward/downward ramps (15° slopes), and step platform for comprehensive navigation testing.
- **default.sdf**: Simple environment with ground plane for basic testing.

## Acknowledgements
Special thanks to [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2) and its contributors. This repository is built upon their excellent work.
