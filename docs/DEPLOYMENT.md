# Production Deployment Guide

This guide covers production deployment of the OM1 ROS2 SDK using Docker.

## Docker Image

We release the Docker image `openmindagi/om1_ros2_sdk`, which provides the full ROS2 system for running the OM1 ROS2 SDK.

## Starting the System

To start all services, run the following commands:

```bash
docker-compose up orchestrator -d --no-build
docker-compose up om1_sensor -d --no-build
docker-compose up watchdog -d --no-build
```

This will bring up the entire system. After the first setup, the system will automatically boot up whenever you start the robot.

## Components Overview

### 1. watchdog

- If any topics or sensors stop publishing data, the watchdog automatically restarts **om1_sensor**.
- Ensures system stability during long-running sessions.

### 2. om1_sensor

The **om1_sensor** container manages all low-level sensor drivers:

- **Intel RealSense D435** (depth camera)
- **RPLidar** (LiDAR scanning)

It publishes ROS2 topics consumed by the rest of the system:

- `/om/paths` — Processed path and localization data
- `/scan` — Raw LiDAR scan data

### 3. orchestrator

The **orchestrator** service provides an API endpoint and cloud service integration.

It manages:

- SLAM (Simultaneous Localization and Mapping)
- Navigation (**Nav2**)
- Map storage and loading

You can interact with it via REST APIs.

## API Usage

### Start SLAM

```bash
curl --location 'http://localhost:5000/start/slam' \
     --header 'Content-Type: application/json' \
     --data '{}'
```

### Stop SLAM

```bash
curl --location 'http://localhost:5000/stop/slam' \
     --header 'Content-Type: application/json' \
     --data '{}'
```

### Save a Map

```bash
curl --location 'http://localhost:5000/maps/save' \
     --header 'Content-Type: application/json' \
     --data '{"map_name": "office"}'
```

### Start Navigation (Nav2)

```bash
curl --location 'http://localhost:5000/start/nav2' \
     --header 'Content-Type: application/json' \
     --data '{"map_name": "maps/office/office.yaml"}'
```

### Stop Navigation (Nav2)

```bash
curl --location 'http://localhost:5000/stop/nav2' \
     --header 'Content-Type: application/json' \
     --data '{}'
```
