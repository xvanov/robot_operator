import rclpy
from bb_msgs.srv import OperationLaunch
from rclpy.node import Node
from std_msgs.msg import String

from robot_operation.constants import LOGGER_PUBLISHER_QUEUE_SIZE


class RobotCommandClient(Node):
    """A client node for sending commands to a robot via a ROS2 service.

    This client node creates a service client for the 'robot_command' service,
    which is intended to handle robot operation commands.
    """

    def __init__(self):
        """Initialize the robot command client node and create a service client."""
        super().__init__("robot_command_client")
        self.client = self.create_client(
            OperationLaunch,
            "robot_command",
        )
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def send_command(self, command):
        """Send a command to the robot command service.

        Args:
            command (str): The command to send to the robot.
        """
        request = OperationLaunch.Request()
        request.command = command

        self.future = self.client.call_async(request)


class FullMessagePrinter(Node):
    """A ROS2 subscriber that prints out full echo of publisher topic.

    Attributes:
        subscription (Subscription): The subscription to the ROS topic.
    """

    def __init__(self, log_topic_name):
        """Initializes the FullMessagePrinter subscription to a given topic."""
        super().__init__("robot_log_printer")
        self.commander_subscription = self.create_subscription(
            String,
            log_topic_name,
            self.listener_callback,
            LOGGER_PUBLISHER_QUEUE_SIZE,
        )

    def listener_callback(self, msg):
        """Function called whenever a message is received on a topic.

        Args:
            msg (String): The received message object.
        """
        self.get_logger().info('"%s"' % msg.data)


def launch_commander_log_printer(args=None):
    """Launches the robot_log_printer node for echoing robot log messages.

    Args:
        args: Arguments passed to rclpy.init(). Defaults to None.
    """
    rclpy.init(args=args)
    node = FullMessagePrinter("/commander_logs")
    rclpy.spin(node)
    rclpy.shutdown()


def launch_scheduler_log_printer(args=None):
    """Launches the robot_log_printer node for echoing robot log messages.

    Args:
        args: Arguments passed to rclpy.init(). Defaults to None.
    """
    rclpy.init(args=args)
    node = FullMessagePrinter("/scheduler_logs")
    rclpy.spin(node)
    rclpy.shutdown()


def handle_command(robot_command_client, command):
    """Sends a command to the robot and handles the response."""
    robot_command_client.send_command(command)
    while rclpy.ok():
        rclpy.spin_once(robot_command_client)
        if robot_command_client.future.done():
            try:
                response = robot_command_client.future.result()
            except Exception as e:
                robot_command_client.get_logger().info(f"Service call failed {e!r}")
            else:
                robot_command_client.get_logger().info(
                    f"Service response: " f"{response.success!r}",
                )
            break


def prompt_for_command():
    """Prompt the user for a command, returning the command or 'exit'."""
    return input("Enter command or 'exit' to quit: ").strip().lower()


def launch_client(args=None):
    """Main function to run the robot command client."""
    rclpy.init(args=args)
    robot_command_client = RobotCommandClient()

    command = prompt_for_command()
    while command != "exit":
        if command:
            handle_command(robot_command_client, command)
        command = prompt_for_command()

    robot_command_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    launch_client()
