"""The production Global Path node, represented by the `GlobalPath` class.

The node reads a global path from a CSV file (specified by the `global_path_filepath`
parameter), interpolates it so that no two consecutive waypoints are farther apart than
`global_path_interval_spacing_km`, and publishes the result on the `global_path` topic for the
navigate node to consume.

Unlike the development `MockGlobalPath` node (which sources its path from a test plan and
regenerates it from the boat's live GPS position), this node is a pure function of the CSV file:
it does not subscribe to GPS. The path is re-read and re-published whenever the source CSV file
changes on disk, and is re-published every `pub_period_sec` so that subscribers which start after
this node still receive it.

The CSV (and thus the published path) must be ordered final-destination-first: the navigate node
treats waypoint index 0 as the final destination and heads toward it starting from the last
waypoint (see node_navigate.py). This matches the convention used by the development test plans.
"""

import os

import rclpy
from rclpy.node import Node

import custom_interfaces.msg as ci
import local_pathfinding.global_path as gp


def main(args=None):
    rclpy.init(args=args)
    global_path = GlobalPath()

    rclpy.spin(node=global_path)

    global_path.destroy_node()
    rclpy.shutdown()


def get_global_path(file_path: str, interval_spacing_km: float) -> ci.Path:
    """Reads the global path from the constant array and interpolates it so that no two consecutive
    waypoints are farther apart than `interval_spacing_km`.

    Interpolation is anchored to the path's own final waypoint, so the boat's live position is
    not required. The anchor is a separate argument from the path itself: to anchor interpolation
    on the boat's live position instead (adding a lead-in segment from the boat onto the route),
    pass that position as the anchor.

    Args:
        file_path (str): Unused; retained for interface compatibility with the timer callback.
        interval_spacing_km (float): The maximum allowed spacing between waypoints in km.

    Returns:
        ci.Path: The interpolated global path. May be empty if GLOBAL_PATH_WAYPOINTS is empty.
    """
    path = gp.get_path()

    if len(path.waypoints) == 0:
        return path

    # Anchor interpolation to the last waypoint so no boat-relative segment is introduced;
    # interpolation then only fills gaps between the CSV waypoints. write=False keeps this node
    # from writing any csv files (it is a read-only consumer of the global path file).
    anchor = path.waypoints[-1]
    return gp.interpolate_path(
        path=path,
        pos=anchor,
        interval_spacing=interval_spacing_km,
    )


class GlobalPath(Node):
    """Reads a global path from a CSV file and publishes it to the global_path topic.

    Publisher:
        global_path_pub (Publisher): Publishes a `Path` msg containing the global path.

    Attributes:
        global_path (ci.Path): The most recently published global path, or None if no valid path
            has been loaded yet.
        file_mtime (float): The modification time of the CSV file when it was last loaded.

    Parameters: see [Sailbot ROS Parameter Configuration](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/global_launch/config/README.md)  # noqa: E501
        for their documentation
    """

    def __init__(self):
        super().__init__(node_name="global_path")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("global_path_filepath", rclpy.Parameter.Type.STRING),
                ("global_path_interval_spacing_km", rclpy.Parameter.Type.DOUBLE),
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.global_path_pub = self.create_publisher(
            msg_type=ci.Path, topic="global_path", qos_profile=10
        )

        self.global_path = None
        self.file_mtime = None
        self._load_error_logged = False

        pub_period_sec = self.get_parameter("pub_period_sec").get_parameter_value().double_value

        # Load and publish immediately so subscribers do not have to wait a full period.
        self.publish_global_path()

        self.timer = self.create_timer(
            timer_period_sec=pub_period_sec, callback=self.publish_global_path
        )

    def publish_global_path(self):
        """Reload the global path from the CSV if the file changed, then publish it.

        Publishing on every timer tick ensures that subscribers which start after this node still
        receive the (unchanged) global path, without coupling re-publishing to GPS. Republishing
        an identical path is harmless: the navigate node compares waypoint values, so it does not
        treat an unchanged path as a new global waypoint. An empty or unreadable path is never
        published, since downstream consumers (LocalPathState) require a non-empty global path.
        """
        self._reload_if_changed()

        if self.global_path is not None:
            self.global_path_pub.publish(self.global_path)
            self.get_logger().debug(f"Publishing global path: {gp.path_to_dict(self.global_path)}")

    def _reload_if_changed(self):
        """Reloads and interpolates the global path from the CSV file if the file has changed
        since it was last loaded. Leaves the last valid path in place if reading fails."""
        file_path = self.get_parameter("global_path_filepath").get_parameter_value().string_value

        try:
            mtime = os.path.getmtime(file_path)
        except OSError:
            self._log_load_error_once(f"Global path file not found or unreadable: {file_path}")
            return

        if self.global_path is not None and mtime == self.file_mtime:
            return  # already loaded this version of the file

        interval_spacing_km = (
            self.get_parameter("global_path_interval_spacing_km")
            .get_parameter_value()
            .double_value
        )

        try:
            path = get_global_path(file_path, interval_spacing_km)
        except Exception as e:
            self._log_load_error_once(f"Failed to read global path from {file_path}: {e}")
            return

        if len(path.waypoints) == 0:
            self._log_load_error_once(
                f"Global path file {file_path} contains no waypoints; not publishing"
            )
            return

        self.file_mtime = mtime
        self.global_path = path
        self._load_error_logged = False
        waypoints = self.global_path.waypoints
        self.get_logger().info(
            f"Loaded global path from {file_path}: {len(waypoints)} waypoints. "
            f"navigate sails toward index 0 (final destination) "
            f"({waypoints[0].latitude:.4f}, {waypoints[0].longitude:.4f}), starting from the "
            f"last waypoint ({waypoints[-1].latitude:.4f}, {waypoints[-1].longitude:.4f})."
        )
        self.get_logger().debug(f"Global path waypoints: {gp.path_to_dict(self.global_path)}")

    def _log_load_error_once(self, message: str):
        """Logs an error once per error state, to avoid spamming on every timer tick while the
        file remains missing or invalid. The flag is reset on a successful load."""
        if not self._load_error_logged:
            self.get_logger().error(message)
            self._load_error_logged = True


if __name__ == "__main__":
    main()
