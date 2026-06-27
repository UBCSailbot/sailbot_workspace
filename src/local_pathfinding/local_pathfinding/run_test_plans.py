"""Sequential runner for local pathfinding test plans."""

from __future__ import annotations

import argparse
import json
import os
import random
import signal
import subprocess
import time
import yaml
import rclpy

from rclpy.node import Node
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs


DEFAULT_TEST_COUNT = 5
DEFAULT_TIMEOUT_HOURS = 5.0
DEFAULT_GOAL_THRESHOLD_M = 300.0
STATUS_UPDATE_INTERVAL_SEC = 1.0
EXCLUDED_TEST_PLANS = {"on_water_mock_ais.yaml"}


@dataclass(frozen=True)
class TestPlanInfo:
    index: int
    name: str
    path: Path
    global_waypoints: tuple[ci.HelperLatLon, ...]


def parse_waypoints(path: Path) -> tuple[ci.HelperLatLon, ...]:
    """Read the global waypoint list from a test plan's YAML file.
    """
    with path.open("r") as file:
        data = yaml.safe_load(file)
    data = data or {}
    global_path = data.get("global_path")
    global_waypoints = global_path["waypoints"]

    parsed_waypoints: list[ci.HelperLatLon] = []
    for waypoint in global_waypoints:
        parsed_waypoints.append(
            ci.HelperLatLon(
                latitude=float(waypoint["latitude"]),
                longitude=float(waypoint["longitude"]),
            )
        )
    return tuple(parsed_waypoints)


def get_test_plans(test_plans_dir: Path) -> list[TestPlanInfo]:
    """
    Get the list of all the test plans in the given directory, excluding any that are in
    EXCLUDED_TEST_PLANS.
    """
    runnable_plans: list[TestPlanInfo] = []
    for path in sorted(test_plans_dir.glob("*.yaml")):
        if path.name in EXCLUDED_TEST_PLANS:
            continue
        runnable_plans.append(
            TestPlanInfo(
                index=len(runnable_plans) + 1,
                name=path.name,
                path=path,
                global_waypoints=parse_waypoints(path),
            )
        )
    return runnable_plans


def select_plans(plans: list[TestPlanInfo], user_input: list[str]) -> list[TestPlanInfo]:
    """
    Makes a list of plans selected by user input, which can be a list of test numbers or names
    separated by commas or spaces.
    Raises ValueError if any input is invalid.

    Returns: A list of TestPlanInfo objects corresponding to the selected plans.
    """
    #  Parsing user input into a list of str
    user_selected: list[str] = []
    user_input = ",".join(user_input).split(",")
    for item in user_input:
        item = item.strip()
        if item:
            user_selected.append(item)
    #  Selecting plans based on user input
    selected: list[TestPlanInfo] = []
    selected_names: set[str] = set()

    by_index = {str(plan.index): plan for plan in plans}
    by_name = {plan.name: plan for plan in plans}
    by_stem = {plan.path.stem: plan for plan in plans}

    for item in user_selected:
        plan = by_index.get(item) or by_name.get(item) or by_stem.get(item)
        if plan is None:
            valid_names = ", ".join(plan.name for plan in plans)
            raise ValueError(f"Unknown test selected: {item}. Valid tests: {valid_names}")
        if plan.name not in selected_names:
            selected.append(plan)
            selected_names.add(plan.name)
    return selected


def resolve_test_plans(
    plans: list[TestPlanInfo],
    selected: list[TestPlanInfo],
    target_count: int,
    seed: int | None,
) -> list[TestPlanInfo]:
    """
    Makes the final list of test plans to run, filling in any unselected slots with randomly
    selected plans from the remaining runnable plans.
    Raises ValueError if target_count is invalid.

    Returns: A list of TestPlanInfo objects corresponding to the selected plans.
    """
    if target_count < 1 or target_count > len(plans):
        raise ValueError(f"--num_tests must be between 1 and {len(plans)}")
    if len(selected) > target_count:
        raise ValueError(f"Selected {len(selected)} tests, but --num_tests is {target_count}. "
                         "Increase --num_tests or select fewer tests.")
    random_seed = random.Random(seed)
    selected_test_names = {plan.name for plan in selected}
    remaining_test_names = [plan for plan in plans if plan.name not in selected_test_names]
    fill_count = target_count - len(selected)
    if fill_count == 0:
        return selected
    return selected + random_seed.sample(remaining_test_names, fill_count)


def write_result(result_path: Path, result: dict[str, Any]) -> None:
    """Write test result dictionary to a JSON file.

    The parent directory is created here because each test gets its own output folder, and
    the folder may not exist before that test starts.
    """
    result_path.parent.mkdir(parents=True, exist_ok=True)
    with result_path.open("w") as file:
        json.dump(result, file, indent=2, sort_keys=True)
        file.write("\n")


def build_launch_command(
    args: argparse.Namespace,
    plan: TestPlanInfo,
    save_path: str,
) -> list[str]:
    """Builds the launch command to launch a test plan using ROS2 launch."""
    return [
        "ros2",
        "launch",
        "global_launch",
        "main_launch.py",
        f"mode:={args.mode}",
        "config:=globals.yaml",
        f"log_level:={args.log_level}",
        f"test_plan:={plan.name}",
        "record:=true",
        f"save_path:={save_path}",
    ]


def distance_m(x: ci.HelperLatLon, y: ci.HelperLatLon) -> float:
    """Return the geodesic distance between two latitude/longitude points in meters."""
    _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
        x.longitude, x.latitude, y.longitude, y.latitude
    )
    return float(distance_to_waypoint_m)


def remaining_total_distance(
    boat_position: ci.HelperLatLon,
    waypoints: tuple[ci.HelperLatLon, ...],
    current_waypoint_index: int
) -> float:
    """
    Returns the total distance (in meters) from the boat's current position to the final global
    waypoint, following all the global waypoints in order.
    """
    if current_waypoint_index < 0 or current_waypoint_index >= len(waypoints):
        return 0.0
    total_dist = distance_m(boat_position, waypoints[current_waypoint_index])
    for waypoint_index in range(current_waypoint_index, 0, -1):
        total_dist += distance_m(waypoints[waypoint_index], waypoints[waypoint_index - 1])
    return total_dist


def make_goal_monitor_node(plan: TestPlanInfo, goal_threshold_m: float):
    """Create a temporary ROS node that watches GPS progress for one test plan.

    The node subscribes to /gps, compares the boat position to the active global waypoint,
    and marks the test complete once the boat reaches the final global waypoint. A waypoint
    is considered reached when the boat is within goal_threshold_m meters of it.
    """
    class GoalMonitor(Node):
        def __init__(self):
            node_name = f"test_plan_goal_monitor_{plan.path.stem}"
            super().__init__(node_name=node_name)
            self.current_waypoint_index = len(plan.global_waypoints) - 1
            self.completed = False
            self.last_distance_m: float | None = None
            self.remaining_route_distance_m: float | None = None
            self.boat_speed_kmph: float | None = None
            self.remaining_local_waypoints: int | None = None
            self.last_gps_at_monotonic: float | None = None
            self._gps_sub = self.create_subscription(
                msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
            )
            self._local_path_sub = self.create_subscription(
                msg_type=ci.LPathData,
                topic="local_path",
                callback=self.local_path_callback,
                qos_profile=10
            )

        def local_path_callback(self, msg: ci.LPathData) -> None:
            self.remaining_local_waypoints = int(msg.remaining_waypoints)

        def gps_callback(self, msg: ci.GPS) -> None:
            boat_latlon = ci.HelperLatLon(latitude=msg.lat_lon.latitude,
                                          longitude=msg.lat_lon.longitude)
            self.last_gps_at_monotonic = time.monotonic()
            self.boat_speed_kmph = float(msg.speed.speed)

            while self.current_waypoint_index >= 0:
                target_waypoint = plan.global_waypoints[self.current_waypoint_index]
                self.last_distance_m = distance_m(boat_latlon, target_waypoint)
                if self.last_distance_m >= goal_threshold_m:
                    self.remaining_route_distance_m = remaining_total_distance(
                        boat_latlon,
                        plan.global_waypoints,
                        self.current_waypoint_index
                    )
                    return
                self.get_logger().debug(
                    f"Reached waypoint index {self.current_waypoint_index} "
                    f"for {plan.name} within {self.last_distance_m:.1f} m"
                )
                if self.current_waypoint_index == 0:
                    self.completed = True
                    self.remaining_route_distance_m = 0.0
                    return
                self.current_waypoint_index -= 1
            self.last_distance_m = 0.0
            self.remaining_route_distance_m = 0.0
    return GoalMonitor()


def stop_process_group(process: subprocess.Popen, grace_sec: float = 15.0) -> None:
    """Stops a ros2 launch process and all child node processes."""

    if process.poll() is not None:
        return

    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=grace_sec)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait()


def format_duration(seconds: float) -> str:
    """Format the duration in seconds as HH:MM:SS."""
    seconds = max(0, int(seconds))
    hours, remainder = divmod(seconds, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def format_distance(distance_meters: float | None) -> str:
    """Format the distance in meters, using kilometers for distances of at least one kilometer."""
    if distance_meters is None:
        return "--"
    if distance_meters >= 1000:
        return f"{distance_meters / 1000:.2f} km"
    return f"{distance_meters:.0f} m"


def format_speed(speed_kmph: float | None) -> str:
    """Format a speed in kilometers per hour."""
    if speed_kmph is None:
        return "--"
    return f"{speed_kmph:.2f} km/h"


def update_progress_display(
    plan: TestPlanInfo,
    monitor: Any,
    test_number: int,
    total_tests: int,
    timeout_sec: float,
    test_start_monotonic_sec: float,
    curr_monotonic_sec: float,
    progress_update_monotonic_sec: float,
    previous_status_length: int,
) -> tuple[float, int]:
    """
    Updates the progress line when the status update interval has elapsed.

    Returns the most recent progress update time and status line length for use in the next
    update.
    """
    if (curr_monotonic_sec - progress_update_monotonic_sec <= STATUS_UPDATE_INTERVAL_SEC):
        return progress_update_monotonic_sec, previous_status_length

    elapsed_time = curr_monotonic_sec - test_start_monotonic_sec
    remaining_global_waypoints = max(monitor.current_waypoint_index + 1, 0)
    status_line = (
        f"[{test_number}/{total_tests}] {plan.name} | "
        f"Elapsed: {format_duration(elapsed_time)} / {format_duration(timeout_sec)} | "
        f"Remaining Local Waypoints: {monitor.remaining_local_waypoints or 0} | "
        f"Remaining Global Waypoints: {remaining_global_waypoints} / "
        f"{len(plan.global_waypoints)}| "
        f"Active Global Waypoint Dist.: {format_distance(monitor.last_distance_m)} | "
        f"Total Route left: {format_distance(monitor.remaining_route_distance_m)} | "
        f"Boat Speed: {format_speed(monitor.boat_speed_kmph)}"
    )
    clear_padding = " " * max(previous_status_length - len(status_line), 0)
    print(f"\r{status_line}{clear_padding}", end="", flush=True)

    return curr_monotonic_sec, len(status_line)


def run_test_plan(
    args: argparse.Namespace,
    workspace: Path,
    batch_id: str,
    plan: TestPlanInfo,
    test_number: int,
    total_tests: int,
) -> dict[str, Any]:
    """
    Runs a single test plan and returns the result as a dictionary.
    Arguments:
        args: The command line arguments.
        workspace: The ROS workspace path.
        batch_id: The unique identifier for this batch of tests.
        plan: The TestPlanInfo object for the test plan to run.
        test_number: The index of this test in the batch (1-based).
        total_tests: The total number of tests in the batch.
    Returns:
        A dictionary object containing the result of the test plan execution, including status and
        any relevant information such as the test plan name, batch ID, and test number.
    """
    parts = [args.save_path.strip("/"), batch_id, plan.path.stem]
    save_path = "/".join(part for part in parts if part)
    launch_command = build_launch_command(args, plan, save_path)
    result: dict[str, Any] = {}
    result_dir = workspace / save_path
    result_path = result_dir / "result.json"
    ros_launch_log_path = result_dir / "launch.log"

    timeout_sec = args.timeout_hours * 3600
    test_plan_start_time = datetime.now().isoformat(timespec="seconds")
    test_start_monotonic_sec = time.monotonic()

    result_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nStarting {test_number}/{total_tests}: {plan.name}")
    print(f"Output directory: {result_dir}")
    print("Visualizer_DashApp: http://127.0.0.1:8050/")

    monitor = make_goal_monitor_node(plan, args.goal_threshold_m)

    status = "UNKNOWN"
    reason = ""
    return_code: int | None = None
    progress_update_monotonic_sec = 0.0
    previous_status_length = 0

    try:
        with ros_launch_log_path.open("w", buffering=1) as ros_launch_log:
            launch_process = subprocess.Popen(
                launch_command,
                stdout=ros_launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            while True:
                rclpy.spin_once(monitor, timeout_sec=1.0)
                return_code = launch_process.poll()
                curr_monotonic_sec = time.monotonic()

                if monitor.completed:
                    status = "COMPLETED"
                    reason = "Reached final Global waypoint before timeout."
                    stop_process_group(launch_process)
                    return_code = launch_process.poll()
                    break

                if return_code is not None:
                    status = "FAILED" if return_code != 0 else "EXITED"
                    reason = (
                        f"Launch exited with code {return_code}" +
                        " before reaching final Global waypoint."
                    )
                    break

                if (curr_monotonic_sec - test_start_monotonic_sec) > timeout_sec:
                    status = "TIMEOUT"
                    reason = f"Test plan exceeded timeout of {args.timeout_hours:g} hours."
                    stop_process_group(launch_process)
                    return_code = launch_process.poll()
                    break

                progress_update_monotonic_sec, previous_status_length = (
                    update_progress_display(
                        plan=plan,
                        monitor=monitor,
                        test_number=test_number,
                        total_tests=total_tests,
                        timeout_sec=timeout_sec,
                        test_start_monotonic_sec=test_start_monotonic_sec,
                        curr_monotonic_sec=curr_monotonic_sec,
                        progress_update_monotonic_sec=progress_update_monotonic_sec,
                        previous_status_length=previous_status_length,
                    )
                )
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Terminating test plan...")
        status = "INTERRUPTED"
        reason = "Interrupted by user"
        if launch_process is not None:
            stop_process_group(launch_process)
            return_code = launch_process.poll()
    finally:
        monitor.destroy_node()
    ended_at = datetime.now().isoformat(timespec="seconds")
    duration_sec = time.monotonic() - test_start_monotonic_sec
    result = {
        "test": plan.name,
        "status": status,
        "reason": reason,
        "test_plan_start_time": test_plan_start_time,
        "ended_at": ended_at,
        "duration_sec": round(duration_sec, 3),
        "return_code": return_code,
        "save_path": save_path,
        "result_path": str(result_path),
        "ros_launch_log_path": str(ros_launch_log_path),
        "ros_launch_log_stored": status != "COMPLETED",
        "last_target_waypoint_index": monitor.current_waypoint_index,
        "last_distance_m": monitor.last_distance_m,
        "remaining_route_distance_m": monitor.remaining_route_distance_m,
        "remaining_local_waypoints": monitor.remaining_local_waypoints,
        "command": launch_command,
    }
    write_result(result_path, result)
    if status == "COMPLETED":
        try:
            ros_launch_log_path.unlink()
        except FileNotFoundError:
            pass
    else:
        print(f"Ros Launch log saved to: {ros_launch_log_path}")
    print(f"\nFinished {test_number}/{total_tests}: {plan.name} -> {status}")
    print(f"Reason: {reason}")
    print(f"Result: {result_path}")
    return result


def print_plan_list(plans: list[TestPlanInfo]) -> None:
    """
    Prints the list of test plans and any excluded test plans.
    """

    print("Runnable test plans:")
    for plan in plans:
        print(f"  {plan.index:2d}. {plan.name}")

    if EXCLUDED_TEST_PLANS:
        print("\nExcluded test plans:")
        for name in EXCLUDED_TEST_PLANS:
            print(f"  - {name}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run local pathfinding test plans sequentially.",
    )
    parser.add_argument(
        "-t",
        "--tests",
        nargs="*",
        default=[],
        help="Test numbers or names to run. Supports comma/space-separated values.",
    )
    parser.add_argument(
        "-n",
        "--num_tests",
        type=int,
        default=None,
        help=(
            "Total number of tests to run. Defaults to 5, filling unselected slots randomly. "
            "Can be any value from 1 to the number of runnable tests."
        ),
    )
    parser.add_argument("--seed", type=int, default=None, help="Random seed for fill selection.")
    parser.add_argument("--list", action="store_true", help="List runnable test plans and exit.")
    parser.add_argument(
        "--mode",
        default="development",
        help="Launch mode to pass to global_launch.",
    )
    parser.add_argument(
        "--log_level",
        default="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        help="ROS log level to pass to global_launch.",
    )
    parser.add_argument(
        "--timeout_hours",
        type=float,
        default=DEFAULT_TIMEOUT_HOURS,
        help="Per-test timeout in hours.",
    )
    parser.add_argument(
        "--goal_threshold_m",
        type=float,
        default=DEFAULT_GOAL_THRESHOLD_M,
        help="Distance thresholdfrom a waypoint that counts as reached.",
    )
    parser.add_argument(
        "--save_path",
        default=os.path.join(
            "notebooks", "local_pathfinding", "session_recordings", "test_plans_results"),
        help="Workspace-relative root directory for recordings and result files.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workspace = Path(os.getenv("ROS_WORKSPACE", Path(__file__).resolve().parents[3]))
    test_plans_dir = workspace / "src" / "local_pathfinding" / "test_plans"

    if Path(args.save_path).is_absolute():
        raise SystemExit("--save_path must be relative to the workspace.")
    if args.timeout_hours <= 0:
        raise SystemExit("--timeout_hours must be greater than 0.")
    if args.goal_threshold_m <= 0:
        raise SystemExit("--goal_threshold_m must be greater than 0.")

    plans = get_test_plans(test_plans_dir)
    if not plans:
        raise SystemExit(f"No runnable test plans found in {test_plans_dir}")

    if args.list:
        print_plan_list(plans)
        return 0

    try:
        user_selected_plans = select_plans(plans, args.tests)
        target_count = (
            args.num_tests if args.num_tests is not None else min(DEFAULT_TEST_COUNT, len(plans))
        )
        plans_to_run = resolve_test_plans(plans, user_selected_plans, target_count, args.seed)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    batch_id = datetime.now().strftime("%Y_%m_%d-%H_%M_%S_%f")
    print("Selected test plans:")
    for plan in plans_to_run:
        source = "selected" if plan in user_selected_plans else "random"
        print(f"  - {plan.name} ({source})")
    print(f"Batch id: {batch_id}")

    rclpy.init(args=None)
    results: list[dict[str, Any]] = []
    try:
        for test_number, plan in enumerate(plans_to_run, start=1):
            result = run_test_plan(args, workspace, batch_id, plan, test_number, len(plans_to_run))
            results.append(result)
            if result["status"] == "INTERRUPTED":
                print("Test run interrupted by user. Stopping further tests.")
                break
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    summary_path = workspace / args.save_path / batch_id / "summary.json"
    write_result(summary_path, {"batch_id": batch_id, "results": results})
    print(f"\nSummary: {summary_path}")
    return 0 if all(result["status"] == "COMPLETED" for result in results) else 1


if __name__ == "__main__":
    main()
