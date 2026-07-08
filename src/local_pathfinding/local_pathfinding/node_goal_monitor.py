"""ROS node for monitoring test plan goal progress."""

from __future__ import annotations

import time

from rclpy.node import Node

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs


def distance_m(x: ci.HelperLatLon, y: ci.HelperLatLon) -> float:
    """Return the geodesic distance between two latitude/longitude points in meters."""
    _, _, distance_to_waypoint_m = cs.GEODESIC.inv(
        x.longitude, x.latitude, y.longitude, y.latitude
    )
    return float(distance_to_waypoint_m)


def remaining_total_distance(
    boat_position: ci.HelperLatLon,
    waypoints: tuple[ci.HelperLatLon, ...],
    current_waypoint_index: int,
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


class GoalMonitor(Node):
    """Watch GPS progress for one test plan."""

    def __init__(
        self,
        node_name: str,
        plan_name: str,
        global_waypoints: tuple[ci.HelperLatLon, ...],
        goal_threshold_m: float,
    ) -> None:
        super().__init__(node_name=node_name)
        self.plan_name = plan_name
        self.global_waypoints = global_waypoints
        self.goal_threshold_m = goal_threshold_m
        self.current_waypoint_index = len(global_waypoints) - 1
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
            qos_profile=10,
        )

    def local_path_callback(self, msg: ci.LPathData) -> None:
        self.remaining_local_waypoints = int(msg.remaining_waypoints)

    def gps_callback(self, msg: ci.GPS) -> None:
        boat_latlon = ci.HelperLatLon(
            latitude=msg.lat_lon.latitude, longitude=msg.lat_lon.longitude
        )
        self.last_gps_at_monotonic = time.monotonic()
        self.boat_speed_kmph = float(msg.speed.speed)

        while self.current_waypoint_index >= 0:
            target_waypoint = self.global_waypoints[self.current_waypoint_index]
            self.last_distance_m = distance_m(boat_latlon, target_waypoint)
            if self.last_distance_m >= self.goal_threshold_m:
                self.remaining_route_distance_m = remaining_total_distance(
                    boat_latlon, self.global_waypoints, self.current_waypoint_index
                )
                return
            self.get_logger().debug(
                f"Reached waypoint index {self.current_waypoint_index} "
                f"for {self.plan_name} within {self.last_distance_m:.1f} m"
            )
            if self.current_waypoint_index == 0:
                self.completed = True
                self.remaining_route_distance_m = 0.0
                return
            self.current_waypoint_index -= 1
        self.last_distance_m = 0.0
        self.remaining_route_distance_m = 0.0
