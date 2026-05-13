import os

import pytest
import yaml

import custom_interfaces.msg as ci
import pytest
from unittest import mock

import local_pathfinding.local_path as lp
import local_pathfinding.node_navigate as nn

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


def test_get_desired_heading_disables_sail_when_path_not_found():
    sailbot = nn.Sailbot.__new__(nn.Sailbot)
    sailbot.gps = mock.Mock()
    sailbot.gps.lat_lon = ci.HelperLatLon(latitude=0.0, longitude=-0.1)
    sailbot.gps.speed = mock.Mock(speed=0.0)
    sailbot.gps.heading = mock.Mock(heading=0.0)
    sailbot.ais_ships = mock.Mock()
    sailbot.ais_ships.ships = []
    sailbot.global_path = ci.Path(
        waypoints=[
            ci.HelperLatLon(latitude=1.0, longitude=1.0),
            ci.HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    sailbot.filtered_wind_sensor = mock.Mock()
    sailbot.filtered_wind_sensor.speed = mock.Mock(speed=5.0)
    sailbot.filtered_wind_sensor.direction = 90
    sailbot.target_lp_wp_index = 1
    sailbot.global_waypoint_index = -1
    sailbot.saved_target_global_waypoint = sailbot.global_path.waypoints[-1]
    sailbot.planner = "rrtstar"
    sailbot.land_multi_polygon = None
    sailbot.get_logger = mock.Mock(return_value=mock.Mock())

    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    sailbot.local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    sailbot.local_path.path = ci.Path(
        waypoints=[
            ci.HelperLatLon(latitude=1.0, longitude=1.0),
            ci.HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    sailbot.local_path.state = lp.LocalPathState(
        gps=sailbot.gps,
        ais_ships=sailbot.ais_ships,
        global_path=sailbot.global_path,
        target_global_waypoint=sailbot.saved_target_global_waypoint,
        filtered_wind_sensor=sailbot.filtered_wind_sensor,
        planner=sailbot.planner,
    )
    sailbot.local_path._ompl_path = mock.Mock()
    sailbot.local_path._ompl_path.get_path.return_value = None

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 0.0
    assert sail is False
    sailbot.get_logger.return_value.warning.assert_called_once_with(
        "Unable to generate a local path; disabling sail"
    )
