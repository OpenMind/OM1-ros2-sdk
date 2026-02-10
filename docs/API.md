# REST API Documentation

The OM1 ROS2 SDK provides a REST API for remote control and monitoring of the robot. The API server runs on port 5000.

## API Endpoints

### GET /api/status

Returns the API status.

**Response:**
```json
{
  "status": "OK",
  "message": "Go2 API is running"
}
```

### GET /api/pose

Returns the current robot pose with covariance.

**Response:**
```json
{
  "position": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "covariance": [...]
}
```

### POST /api/move_to_pose

Sends the robot to a specific pose.

**Request body:**
```json
{
  "position": {"x": 1.0, "y": 2.0, "z": 0.0},
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
}
```

**Response:**
```json
{
  "status": "success",
  "message": "Moving to specified pose"
}
```

### GET /api/amcl_variance

Returns AMCL localization uncertainty.

**Response:**
```json
{
  "x_uncertainty": 0.1,
  "y_uncertainty": 0.1,
  "yaw_uncertainty": 5.0
}
```

### GET /api/nav2_status

Returns the status of active navigation goals.

**Response:**
```json
{
  "nav2_status": [
    {
      "goal_id": "abc123...",
      "status": "EXECUTING",
      "timestamp": {"sec": 1234567890, "nanosec": 123456789}
    }
  ]
}
```

### GET /api/map

Returns the current occupancy grid map.

**Response:**
```json
{
  "map_metadata": {
    "map_load_time": {"sec": 1234567890, "nanosec": 123456789},
    "resolution": 0.05,
    "width": 384,
    "height": 384,
    "origin": {
      "position": {"x": -10.0, "y": -10.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "data": [0, 0, 0, ...]
}
```
