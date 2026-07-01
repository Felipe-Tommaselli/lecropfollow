"""
Shutdown utilities for ROS and Gazebo processes.
"""
import subprocess
import time
from termcolor import colored


def shutdown_ros_and_gazebo():
    """
    Terminate all ROS nodes and Gazebo server.
    Equivalent to: pkill -f ros
    This function is safe to call multiple times.
    """
    print(colored("[shutdown] Stopping ROS and Gazebo processes...", "yellow"))
    try:
        # First try graceful shutdown
        subprocess.run(
            ["pkill", "-f", "ros"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ["pkill", "-f", "gazebo"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        # Wait a moment for graceful shutdown
        print(colored("[shutdown] Waiting for graceful shutdown...", "yellow"))
        time.sleep(3)
        
        # Force kill any remaining processes
        subprocess.run(
            ["pkill", "-9", "-f", "gzserver"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ["pkill", "-9", "-f", "gzclient"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ["pkill", "-9", "-f", "roscore"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        subprocess.run(
            ["pkill", "-9", "-f", "rosmaster"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        
        # Sleep for 12s to guarantee all processes are fully killed
        print(colored("[shutdown] Waiting 12 seconds for complete termination...", "yellow"))
        time.sleep(12)
        print(colored("[shutdown] ROS and Gazebo processes terminated.", "green"))
    except Exception as e:
        print(colored(f"[shutdown] Warning: Failed to kill processes: {e}", "red"))
