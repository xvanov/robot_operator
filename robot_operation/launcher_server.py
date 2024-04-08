import os
import shlex
import sys
import threading
import time
from pathlib import Path

import rclpy
from bb_msgs.srv import OperationLaunch
from rclpy.node import Node
from std_msgs.msg import String  # For publishing log messages
from utils import get_logging_directory

from robot_operation.constants import LOGGER_PUBLISHER_QUEUE_SIZE
from robot_operation.robot_launcher.robot_launcher import (
    clean_up_robot_launch,
    get_launch_commands,
    get_run_commands,
    launch_ros_nodes,
    parse_args,
    run_package_executables,
)


class LauncherServiceServer(Node):
    """A ROS service server node for launching and managing robot operations.

    This node creates a service that can start or stop robot operations based on
    the received commands. It also publishes logs from these operations to a
    specified topic.
    """

    def __init__(self):
        """Initialize the service server, create a service, and setup logging."""
        super().__init__("robot_command_service_server")
        self.service = self.create_service(
            OperationLaunch,
            "robot_command",
            self.execute_callback,
        )
        self.commander_log_publisher = self.create_publisher(
            String,
            "commander_logs",
            LOGGER_PUBLISHER_QUEUE_SIZE,
        )
        self.scheduler_log_publisher = self.create_publisher(
            String,
            "scheduler_logs",
            LOGGER_PUBLISHER_QUEUE_SIZE,
        )
        self.commander_log_file_path = Path(
            get_logging_directory("utils") / ".." / "robot_launcher" / "commander.log",
        )
        self.scheduler_log_file_path = Path(
            get_logging_directory("utils")
            / ".."
            / "robot_launcher"
            / "panel_scheduler.log",
        )
        self.commander_log_file_path.parent.mkdir(exist_ok=True, parents=True)

    def start_robot_operations(self, args):
        """Starts robot operations based on provided arguments.

        Args:
            args (str): A string containing arguments for the robot operations.
        """
        # Convert args string to format suitable for sys.argv
        sys.argv = ["robot_launcher.py"] + shlex.split(args)
        args = parse_args()
        if args.clean_up is not None:
            clean_up_robot_launch(args.clean_up)
        elif args.run is not None:
            run_commands = get_run_commands(args)
            run_package_executables(run_commands)
        else:
            launch_commands = get_launch_commands(args)
            launch_ros_nodes(launch_commands, run_remotely=True)

    def publish_logs(self):
        """Reads the robot operation logs and publishes them to a ROS2 topic."""
        publish_logs(self.commander_log_publisher, self.commander_log_file_path)
        publish_logs(self.scheduler_log_publisher, self.scheduler_log_file_path)

    def execute_callback(self, request, response):
        """Callback for the robot_command service.

        Args:
            request: The request message is an empty string.
            response: "Success" (bool) and "message" (str).

        Returns:
            The response message indicating success or failure.
        """
        self.get_logger().info("Executing command")
        threading.Thread(
            target=self.start_robot_operations,
            args=(request.command,),
        ).start()
        response.success = True
        return response


def publish_logs(log_publisher, log_file_path):
    """Reads the robot operation logs and publishes them to a ROS2 topic."""
    if not os.path.exists(log_file_path):
        # Log file doesn't exist, publish a message indicating so
        log_msg = String()
        log_msg.data = "Log file does not exist."
        log_publisher.publish(log_msg)
    else:
        with open(log_file_path, "r") as log_file:
            log_file.seek(0, 2)  # Move to the end of the file
            last_position = log_file.tell()
            while rclpy.ok():
                log_file.seek(last_position)
                log_content = log_file.read()
                if log_content:
                    log_msg = String()
                    log_msg.data = log_content
                    log_publisher.publish(log_msg)
                last_position = log_file.tell()
                time.sleep(1)


def launch_server(args=None):
    """Main entry point for the ROS service server node.

    Initializes the ROS client, starts the log publishing thread, and spins the
    ROS node to keep it active.
    """
    rclpy.init(args=args)
    service_server = LauncherServiceServer()
    threading.Thread(target=service_server.publish_logs).start()
    rclpy.spin(service_server)
    rclpy.shutdown()


if __name__ == "__main__":
    launch_server()
