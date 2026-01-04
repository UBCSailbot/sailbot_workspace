import custom_interfaces.msg as ci
import pytest

import local_pathfinding.node_navigate as nn
from local_pathfinding.local_path import LocalPath
LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.5
GLOBAL_WAYPOINT_REACHED_THRESH_KM = 3
PATHFINDING_RANGE_KM = 30
ONE_DEGREE_KM = 111  # One degree longitude at equator = 111km
PATH_RANGE_DEG = PATHFINDING_RANGE_KM / ONE_DEGREE_KM


@pytest.mark.parametrize(
    "path, waypoint_index, boat_lat_lon, correct_heading, new_wp_index",
    [
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=-0.1),
            90.0,
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.0),
            180.0,
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.1),
            -135.0,
            0,
        ),
        (
            # Test: boat has reached waypoints[0], heading should be to waypoints[1].
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.1),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=0.09999),
            -90.0,
            1,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index(
    path: ci.Path,
    waypoint_index: int,
    boat_lat_lon: ci.HelperLatLon,
    correct_heading: float,
    new_wp_index: int,
):
    calculated_answer = LocalPath.calculate_desired_heading_and_waypoint_index(
        path, waypoint_index, boat_lat_lon
    )
    assert calculated_answer[0] == pytest.approx(correct_heading, abs=3e-1)
    assert calculated_answer[1] == new_wp_index


@pytest.mark.parametrize(
    "global_path, boat_lat_lon, correct_index",
    [
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            ci.HelperLatLon(latitude=0.0, longitude=-0.1),
            0,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                    ci.HelperLatLon(latitude=0.0, longitude=1 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=2 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=3 * PATH_RANGE_DEG),
                ]
            ),
            ci.HelperLatLon(latitude=0.98 * PATH_RANGE_DEG, longitude=2 * PATH_RANGE_DEG),
            2,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                    ci.HelperLatLon(latitude=0.0, longitude=1 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=2 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=3 * PATH_RANGE_DEG),
                ]
            ),
            ci.HelperLatLon(latitude=1.02 * PATH_RANGE_DEG, longitude=1 * PATH_RANGE_DEG),
            1,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=1.0 * PATH_RANGE_DEG, longitude=1 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=1.0 * PATH_RANGE_DEG, longitude=2 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=2 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=1 * PATH_RANGE_DEG),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            ci.HelperLatLon(latitude=0.98 * PATH_RANGE_DEG, longitude=1 * PATH_RANGE_DEG),
            3,
        ),
    ],
)
def test_find_next_global_waypoint_index(
    global_path: ci.Path, boat_lat_lon: ci.HelperLatLon, correct_index: int
):
    calculated_answer = nn.Sailbot.determine_start_point_in_new_global_path(
        global_path, boat_lat_lon
    )
    assert calculated_answer == correct_index
