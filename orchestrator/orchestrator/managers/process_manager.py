"""
Process lifecycle management for ROS2 launch files.
"""

import os
import signal
import subprocess
from typing import Optional


class ProcessManager:
    """
    ProcessManager handles starting and stopping of ROS2 launch files as subprocesses.
    robot_type is set at construction (from environment variable in orchestrator).
    """

    def __init__(self, robot_type: str = "go2"):
        """
        Initialize the ProcessManager with no active process and robot type.

        robot_type: "go2" or "g1" (set at orchestrator startup)

        Parameters
        ----------
        robot_type : str
            The type of robot, either "go2", "g1" or "tron". This determines which ROS2 package's launch files to use.
        """
        self.process: Optional[subprocess.Popen] = None
        self.robot_type = robot_type

        self.package_name = "go2_sdk"
        if self.robot_type == "g1":
            self.package_name = "g1_sdk"
        elif self.robot_type == "tron":
            self.package_name = "tron_sdk"
        else:
            raise ValueError(f"Unsupported robot type: {self.robot_type}")

    def start(self, launch_file: str, args: Optional[str] = None) -> bool:
        """
        Start a ROS2 launch file as a subprocess.

        Parameters
        ----------
        launch_file : str
            The name of the launch file to run (e.g., 'slam_launch.py' or 'nav2_launch.py').
        args : Optional[str]
            extra arguments to pass to the launch file (e.g., map yaml file).

        Returns
        -------
        bool
            True if the process was started successfully, False if a process is already running.
        """
        if self.process is None or self.process.poll() is not None:
            cmd = ["ros2", "launch", self.package_name, launch_file]
            if args:
                cmd.extend(args.split())
            self.process = subprocess.Popen(cmd, preexec_fn=os.setsid)
            return True
        return False

    def stop(self) -> bool:
        """
        Stop the currently running subprocess.

        Returns
        -------
        bool
            True if the process was stopped successfully, False if no process
            was running.
        """
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
                self.process = None
                return True
            except (ProcessLookupError, OSError, subprocess.TimeoutExpired):
                if self.process:
                    try:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass
                self.process = None
                return True
        return False

    def is_running(self) -> bool:
        """
        Check if the process is currently running.

        Returns
        -------
        bool
            True if process is running, False otherwise.
        """
        return self.process is not None and self.process.poll() is None
