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

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs


DEFAULT_TEST_COUNT = 5
DEFAULT_TIMEOUT_HOURS = 5.0
DEFAULT_GOAL_THRESHOLD_M = 300.0
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
    user_selected: list[str] = []
    user_input = ",".join(user_input).split(",")
    for item in user_input:
        item = item.strip()
        if item:
            user_selected.append(item)

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
        f"config:={args.config}",
        f"log_level:={args.log_level}",
        f"test_plan:={plan.name}",
        f"record:={'false' if args.record else 'true'}",
        f"save_path:={save_path}",
    ]


def distance_m(x: ci.HelperLatLon, y: ci.HelperLatLon) -> float:
    """Return the geodesic distance between two latitude/longitude points in meters."""

    _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
        x.longitude, x.latitude, y.longitude, y.latitude
    )
    return float(distance_to_waypoint_m)


def make_goal_monitor_node(plan: TestPlanInfo, goal_threshold_m: float):
    """Create a temporary ROS node that watches GPS progress for one test plan.

    The node subscribes to /gps, compares the boat position to the active global waypoint,
    and marks the test complete once the boat reaches the final global waypoint. A waypoint
    is considered reached when the boat is within goal_threshold_m meters of it.
    """

    class GoalMonitor(Node):
        """Small per-test observer node used by the runner, not by the planner itself."""

        def __init__(self):
            node_name = f"test_plan_goal_monitor_{plan.path.stem}"
            super().__init__(node_name=node_name)
            self.current_waypoint_index = len(plan.global_waypoints) - 1
            self.completed = False
            self.last_distance_m: float | None = None
            self.last_gps_at_monotonic: float | None = None
            self._gps_sub = self.create_subscription(
                msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
            )

        def gps_callback(self, msg: ci.GPS) -> None:
            boat_latlon = ci.HelperLatLon(latitude=msg.lat_lon.latitude,
                                          longitude=msg.lat_lon.longitude)
            self.last_gps_at_monotonic = time.monotonic()

            while self.current_waypoint_index >= 0:
                target_waypoint = plan.global_waypoints[self.current_waypoint_index]
                self.last_distance_m = distance_m(boat_latlon, target_waypoint)
                if self.last_distance_m >= goal_threshold_m:
                    return

                # Test plan global waypoints are followed from the last item toward index 0.
                self.get_logger().debug(
                    f"Reached waypoint index {self.current_waypoint_index} "
                    f"for {plan.name} within {self.last_distance_m:.1f} m"
                )
                if self.current_waypoint_index == 0:
                    self.completed = True
                    return
                self.current_waypoint_index -= 1
            self.last_distance_m = 0.0
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


def run_test_plan(
    args: argparse.Namespace,
    workspace: Path,
    batch_id: str,
    plan: TestPlanInfo,
    test_number: int,
    total_tests: int,
) -> dict[str, Any]:
    result: dict[str, Any] = {}
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
    result_dir = workspace / save_path
    result_path = result_dir / "result.json"

    timeout_sec = args.timeout_hours * 3600
    started_at = datetime.now().isoformat(timespec="seconds")
    started_monotonic = time.monotonic()

    result_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nStarting {test_number}/{total_tests}: {plan.name}")
    print(f"Output directory: {result_dir}")

    monitor = make_goal_monitor_node(plan, args.goal_threshold_m)

    status = "UNKNOWN"
    reason = ""
    return_code: int | None = None

    try:
        launch_process = subprocess.Popen(
                launch_command,
                start_new_session=True,
            )
        while True:
            rclpy.spin_once(monitor, timeout_sec=1.0)
            return_code = launch_process.poll()
            elapsed_sec = time.monotonic() - started_monotonic

            if monitor.completed:
                status = "COMPLETED"
                reason = "Reached final Global waypoint before timeout."
                stop_process_group(launch_process)
                return_code = launch_process.poll()
                break

            if return_code is not None:
                status = "FAILED" if return_code != 0 else "EXITED"
                reason = (
                    f"Launch exited with code {return_code} before reaching final Global waypoint."
                )
                break

            if elapsed_sec > timeout_sec:
                status = "TIMEOUT"
                reason = f"Test plan exceeded timeout of {args.timeout_hours:g} hours."
                stop_process_group(launch_process)
                return_code = launch_process.poll()
                break
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
    duration_sec = time.monotonic() - started_monotonic
    result = {
        "test": plan.name,
        "status": status,
        "reason": reason,
        "started_at": started_at,
        "ended_at": ended_at,
        "duration_sec": round(duration_sec, 3),
        "return_code": return_code,
        "save_path": save_path,
        "result_path": str(result_path),
        "last_distance_m": monitor.last_distance_m,
        "command": launch_command,
    }
    write_result(result_path, result)
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
        "--config",
        default="globals.yaml",
        help="Config file to pass to global_launch.",
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
    parser.add_argument(
        "--record", action="store_true", help="Pass record:=true to global_launch.")
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
            results.append(
                run_test_plan(args, workspace, batch_id, plan, test_number, len(plans_to_run))
            )
    finally:
        rclpy.shutdown()

    summary_path = workspace / args.save_path / batch_id / "summary.json"
    write_result(summary_path, {"batch_id": batch_id, "results": results})
    print(f"\nSummary: {summary_path}")
    return 0 if all(result["status"] == "COMPLETED" for result in results) else 1


if __name__ == "__main__":
    main()
