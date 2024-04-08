import argparse
import subprocess
import time
from enum import Enum

from utils import get_logging_directory, get_wall_panels_dir

# directory to log output to
LOGGING_DIR = get_logging_directory("utils") / ".." / "robot_launcher"
LOGGING_DIR.mkdir(exist_ok=True, parents=True)


class ROSCommand(Enum):
    """Launch or Run commands supported by operator."""

    pass


class LaunchCommand(ROSCommand):
    """Types of launch commands (high level operations) an operator can perform."""

    BB_CONTROL = "bb_control"
    MOVE_GROUP = "move_group"
    IMAGERY_CLIENTS = "imagery_clients"
    COMMANDER = "commander"


class RunCommand(ROSCommand):
    """Supported ROS2 package executables that operator can run."""

    PANEL_SCHEDULER = "panel_scheduler"


class Node(Enum):
    """All nodes spun up to start robot operations.

    Note: Needed to ensure all are killed when this is called with --clean-up flag
    and to enforce starting/killing only processes with these key words.

    TODO: Figure out a better way to stop nodes.
    https://botbuilt.atlassian.net/browse/WP-663
    Using pkill to stop nodes is not ideal, ideally we would use the same
    operations as in LaunchCommand enum to stop processes (that is stop all processes
    associated with one launch file). This is not easily possible, as of today.
    """

    # started only by imagery_clients
    IMAGERY_CLIENTS = "imagery_clients"

    # started only by control
    BB_CONTROL = "bb_control"

    # started only by move_group
    STATIC_TRANSFORM_PUBLISHER = "static_transform_publisher"
    ROBOT_STATE_PUBLISHER = "robot_state_publisher"

    # started by commander and move_group
    BB_COMMANDER = "bb_commander"
    PANEL_WORKCELL = "panel_workcell"
    MOVEIT = "moveit"


launch_command_to_nodes_mapping = {
    LaunchCommand.BB_CONTROL: [Node.BB_CONTROL],
    LaunchCommand.MOVE_GROUP: [
        Node.STATIC_TRANSFORM_PUBLISHER,
        Node.ROBOT_STATE_PUBLISHER,
        Node.BB_COMMANDER,
        Node.PANEL_WORKCELL,
        Node.MOVEIT,
    ],
    LaunchCommand.IMAGERY_CLIENTS: [Node.IMAGERY_CLIENTS],
    LaunchCommand.COMMANDER: [
        Node.STATIC_TRANSFORM_PUBLISHER,
        Node.ROBOT_STATE_PUBLISHER,
        Node.BB_COMMANDER,
        Node.PANEL_WORKCELL,
        Node.MOVEIT,
    ],
}


def node_type(name: str) -> Node:
    """Converts string input to a Node enum instance."""
    try:
        return Node[name.upper()]
    except KeyError:
        raise ValueError(f"{name} is not a valid node name.")


def launch_command_type(name: str) -> LaunchCommand:
    """Converts string input to a LaunchCommand enum instance."""
    try:
        return LaunchCommand[name.upper()]
    except KeyError:
        raise ValueError(f"{name} is not a valid launch command.")


def run_command_type(name: str) -> RunCommand:
    """Converts string input to a RunCommand enum instance."""
    try:
        return RunCommand[name.upper()]
    except KeyError:
        raise ValueError(f"{name} is not a valid run command.")


def command_type(name: str) -> ROSCommand:
    """Converts string input to a Command enum instance."""
    for CommandEnum in (RunCommand, LaunchCommand):
        try:
            return CommandEnum[name.upper()]
        except KeyError:
            continue  # Try the next enum
    # If no command was found
    raise ValueError(f"{name} is not a valid run or launch command.")


def parse_args() -> argparse.Namespace:
    """Parse command line arguments for launching robots.

    Returns:
        argparse.Namespace: Object containing values for passed arguments
    """
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument(
        "-m",
        "--just-moveit-simulation",
        action="store_true",
        help="Run the robots in a MoveIt Simulation. Cannot be combined with `-i`.",
    )
    group.add_argument(
        "-i",
        "--isaac-simulation",
        action="store_true",
        help="Run the robots with Isaac Sim. Cannot be combined with `-m`.",
    )
    parser.add_argument(
        "-u",
        "--user-approval",
        action="store_true",
        help="Ask for user approval before execution actions.",
    )
    parser.add_argument(
        "--clean-up",
        nargs="*",
        type=command_type,
        default=None,
        help=(
            "Cleanup from an existing robot launch."
            "Provide node names to clean specific ones."
        ),
    )
    parser.add_argument(
        "-s",
        "--start-up",
        nargs="*",
        type=launch_command_type,
        default=None,
        help=("Start a specific node."),
    )
    parser.add_argument(
        "-r",
        "--run",
        nargs=1,
        type=run_command_type,
        default=None,
        help=("Run ros2 commands."),
    )
    parser.add_argument(
        "-a",
        "--run_argument",
        nargs="*",
        type=str,
        default=None,
        help=("Optional argument for ros2 commands."),
    )
    parser.add_argument(
        "-d",
        "--development",
        action="store_true",
        help=(
            "Execute panel scheduler in development mode"
            "(don't save results to google sheets)"
        ),
    )
    args = parser.parse_args()
    if args.clean_up == []:
        args.clean_up = list(LaunchCommand) + list(RunCommand)
    if args.start_up == []:
        args.start_up = list(LaunchCommand)
    return args


def get_launch_commands(args: argparse.Namespace) -> list[tuple[str, list[str]]]:
    """Get the commands to launch the ROS nodes based on the command line args.

    Args:
        args (argparse.Namespace): Object containing values for passed arguments

    Returns:
        list[tuple[str, list[str]]]: (name name for logging, [commands to launch node])
    """
    launch_commands = []
    control_command = get_control_command(args)
    if control_command and LaunchCommand.BB_CONTROL in args.start_up:
        launch_commands.append([LaunchCommand.BB_CONTROL.value, control_command])
    if LaunchCommand.MOVE_GROUP in args.start_up:
        launch_commands.append(
            (LaunchCommand.MOVE_GROUP.value, get_move_group_command(args)),
        )
    if LaunchCommand.IMAGERY_CLIENTS in args.start_up:
        launch_commands.append(
            (LaunchCommand.IMAGERY_CLIENTS.value, get_imagery_clients_command()),
        )
    if LaunchCommand.COMMANDER in args.start_up:
        launch_commands.append(
            (LaunchCommand.COMMANDER.value, get_commander_command(args)),
        )
    return launch_commands


def get_run_commands(args: argparse.Namespace) -> list[tuple[str, list[str]]]:
    """Converts run args to full ros2 run commands."""
    run_commands = []
    if RunCommand.PANEL_SCHEDULER in args.run:
        run_commands.append(
            (RunCommand.PANEL_SCHEDULER.value, get_panel_scheduler_command(args)),
        )
    return run_commands


def get_control_command(args: argparse.Namespace) -> list[str] | None:
    """Return the command to launch the control component, if applicable."""
    if not any([args.just_moveit_simulation, args.isaac_simulation]):
        return ["bb_control", "team_launch.py"]


def get_move_group_command(args: argparse.Namespace) -> list[str]:
    """Return the command to launch the move group, handling simulation modes."""
    if not any([args.just_moveit_simulation, args.isaac_simulation]):
        return ["bb_commander", "team_move_group.launch.py"]
    else:
        return ["bb_commander", "fake_team_move_group.launch.py"]


def get_imagery_clients_command() -> list[str]:
    """Return the command to launch the imagery clients."""
    return ["imagery_clients", "stud_plate_connector.launch.py"]


def get_commander_command(args: argparse.Namespace) -> list[str]:
    """Generate and return the commander command."""
    commander_command = ["bb_commander", "commander.launch.py"]
    if args.user_approval:
        commander_command.append("debug_motion:=true")
    else:
        commander_command.append("debug_motion:=false")

    if args.just_moveit_simulation:
        commander_command.append("just_moveit_simulation:=true")
    return commander_command


def get_panel_scheduler_command(args: argparse.Namespace) -> list[str]:
    """Run schedule."""
    panel_scheduler_command = [
        "panel_scheduler",
        "run_schedule",
        args.run[0].value,
        *args.run_argument,
    ]
    if args.development:
        panel_scheduler_command.append("--development-mode")
    else:
        panel_scheduler_command.append("")
    return panel_scheduler_command


def run_package_executables(
    run_commands: list[tuple[str, list[str]]],
) -> list[subprocess.Popen]:
    """Run ROS2 package executable command.

    Args:
        run_commands (list[tuple[str, list[str]]]): The run commands from the
        'get_run_commands' function.

    Returns:
        list[subprocess.Popen]: List of Popen objects from subprocess for each exec
    """
    processes = []
    for file_name, command in run_commands:
        cmd = ["ros2", "run", *command]
        print(cmd)
        output_file = (LOGGING_DIR / f"{file_name}.log").as_posix()
        pipe_cmd = [">>", output_file, "2>&1", "&"]
        proc = subprocess.Popen(
            " ".join(cmd + pipe_cmd),
            shell=True,
        )
        processes.append(proc)
        # sleep for a second to allow execution
        time.sleep(2)
    return processes


def launch_ros_nodes(
    launch_commands: list[tuple[str, list[str]]],
    run_remotely: bool = False,
) -> list[subprocess.Popen]:
    """Launch the required ROS nodes for robot ops.

    Args:
        launch_commands (list[tuple[str, list[str]]]): The launch commands from the
        'get_launch_commands' function.
        run_remotely (bool): Whether to launch all nodes in background, default to false

    Returns:
        list[subprocess.Popen]: List of Popen objects from subprocess for each node
    """
    processes = []
    for file_name, launch_command in launch_commands:
        if file_name == "commander" and not run_remotely:
            bg = "false"
        else:
            bg = "true"
        cmd = ["ros2", "launch", *launch_command, f"run_in_background:={bg}"]
        output_file = (LOGGING_DIR / f"{file_name}.log").as_posix()
        pipe_cmd = [">>", output_file, "2>&1", "&"]
        proc = subprocess.Popen(
            " ".join(cmd + pipe_cmd),
            shell=True,
        )
        processes.append(proc)
        # sleep for a second to allow node to launch
        time.sleep(2)

    if not run_remotely:
        # launch additonal kitty terminal for operator
        bb_ws = (get_wall_panels_dir() / "bb_ws").as_posix()
        kitty_cmd = (
            f"kitty --hold bash -c 'cd {bb_ws}; source install/setup.bash; bash'"
        )
        proc = subprocess.Popen(kitty_cmd, shell=True)

    return processes


def clean_up_robot_launch(launch_command_identifiers: list[LaunchCommand]):
    """Kills nodes spun up by launch command list using pkill.

    Ran when the --clean-up flag is provided.
    """
    for launch_command in launch_command_identifiers:
        for node in launch_command_to_nodes_mapping:
            subprocess.call(["pkill", "-f", node.value])


def main():
    """Program entry point."""
    args = parse_args()
    if args.clean_up is not None:
        clean_up_robot_launch(args.clean_up)
    elif args.run is not None:
        run_commands = get_run_commands(args)
        run_package_executables(run_commands)
    else:
        launch_commands = get_launch_commands(args)
        launch_ros_nodes(launch_commands)


if __name__ == "__main__":
    main()
