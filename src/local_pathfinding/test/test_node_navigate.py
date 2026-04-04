import os

import pytest
import yaml

import custom_interfaces.msg as ci
import local_pathfinding.node_navigate as nn
from local_pathfinding.local_path import LocalPath

LOCAL_WAYPOINT_REACHED_THRESH_KM = 0.5
GLOBAL_WAYPOINT_REACHED_THRESH_KM = 3
ONE_DEGREE_KM = 111  # One degree longitude at equator = 111km
with open(os.getcwd() + "/../global_launch/config/globals.yaml", "r") as f:
    config = yaml.safe_load(f)
GLOBAL_PATH_SPACING_KM = config["/**"]["ros__parameters"]["global_path_interval_spacing_km"]
PATH_RANGE_DEG = GLOBAL_PATH_SPACING_KM / ONE_DEGREE_KM


@pytest.mark.parametrize(
    "path, target_wp_index, boat_lat_lon, correct_heading, new_target_wp_index",
    [
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=1.0, longitude=1.0),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            ci.HelperLatLon(latitude=0.0, longitude=-0.1),
            90.0,
            1,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=1.0, longitude=1.0),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            ci.HelperLatLon(latitude=0.1, longitude=0.0),
            180.0,
            1,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=1.0, longitude=1.0),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            ci.HelperLatLon(latitude=0.1, longitude=0.1),
            -135.0,
            1,
        ),
        (
            # Test: boat has reached waypoints[1], heading should be to waypoints[2].
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.2),
                    ci.HelperLatLon(latitude=0.0, longitude=0.1),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            ci.HelperLatLon(latitude=0.0, longitude=0.09999),
            -90.0,
            2,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index(
    path: ci.Path,
    target_wp_index: int,
    boat_lat_lon: ci.HelperLatLon,
    correct_heading: float,
    new_target_wp_index: int,
):

    calculated_answer = LocalPath.calculate_desired_heading_and_wp_index(
        path, target_wp_index, boat_lat_lon
    )

    assert calculated_answer[0] == pytest.approx(correct_heading, abs=3e-1)
    assert calculated_answer[1] == new_target_wp_index


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
        global_path, boat_lat_lon, GLOBAL_PATH_SPACING_KM
    )
    assert calculated_answer == correct_index
