# Unitree Go2 Isaac Sim (go2_isaac_sim)

This repository contains the ROS 2 Isaac Sim script for the Unitree Go2 quadruped robot. It provides a complete simulation environment compatible with the unitree-sdk, including mapping and navigation capabilities 

## Features
- **Isaac Sim**: Realistic physics simulation of the Unitree Go2.
- **Navigation Stack (Nav2)**: Fully configured navigation stack for autonomous movement.
- **SLAM**: Mapping capabilities using `slam_toolbox`.
- **LiDAR Support**: Simulation of Velodyne VLP-16 and Unitree 4D LiDAR.

## Installation
  To install Isaac Isaac Sim, running the following commands, or check the instructions [here](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/index.html) to install Isaac Sim.

  ```bash
  uv venv --python 3.11 --seed env_isaacsim

  source env_isaacsim/bin/activate

  # note that here we are installing IsaacSim 5.1
  pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com

  # install the following or another CUDA-enabled PyTorch build that matches your system architecture

  pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128

  # after installation, run the following to test successful installation
  isaacsim
  ```

## Running
  To run the script, simply do (note: a trained policy is required, which should contain the policy.pt, env.yaml, and deploy.yaml files)
  ```bash
  source env_isaacsim/bin/activate
  python3 run.py --policy_dir YOUR_POLICY_DIR
  ```
