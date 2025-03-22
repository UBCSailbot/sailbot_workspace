import logging

import pytest
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor

import local_pathfinding.local_path as local_path

# Configure the logger for testing purposes
logging.basicConfig(level=logging.DEBUG)
test_logger = logging.getLogger("test_logger")

# Initialize the LocalPath instance with the test logger
PATH = local_path.LocalPath(parent_logger=test_logger)


@pytest.mark.parametrize(
    "gps, ais_ships, global_path, filtered_wind_sensor, planner",
    [
        (GPS(), AISShips(), Path(), WindSensor(), "rrtstar"),
    ],
)
def test_LocalPath_update_if_needed(gps, ais_ships, global_path, filtered_wind_sensor, planner):
    PATH.update_if_needed(
        gps=gps,
        ais_ships=ais_ships,
        global_path=global_path,
        filtered_wind_sensor=filtered_wind_sensor,
        planner=planner,
    )
    assert (
        PATH.waypoints is None
    ), "waypoints is not initialized"  # Waypoint should be none as wind speed = 0.0


@pytest.mark.parametrize(
    "gps_lat, gps_lon, waypoints, buffer_distance, expected_result",
    [
        (
            48.3898917,
            -125.0318278,
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            False,
        ),
        (
            48.3775278,
            -124.9368167,
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            False,
        ),
        (
            48.4439528,
            -124.9530528,
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            True,
        ),
        (
            48.5755778,
            -124.8806528,
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            False,
        ),
        (
            48.5136611,
            -124.6219000,
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            False,
        ),
    ],
)
def test_LocalPath_sailbot_drifted_from_old_path(
    gps_lat, gps_lon, waypoints, buffer_distance, expected_result
):
    gps = GPS()
    gps.lat_lon = HelperLatLon(latitude=gps_lat, longitude=gps_lon)

    # Create LocalPath instance with the test logger
    path_instance = local_path.LocalPath(parent_logger=test_logger)
    path_instance.waypoints = waypoints

    result = path_instance.sailbot_drifted_from_old_path(
        gps=gps, waypoints=waypoints, buffer=buffer_distance
    )
    assert (
        result == expected_result
    ), f"Test failed for GPS ({gps_lat}, {gps_lon}), expected {expected_result} but got {result}"


@pytest.mark.parametrize(
    "global_path_points, waypoints, buffer_distance, expected_result",
    [
        (
            [
                (48.5487833, -125.1470583),
                (48.4804222, -125.0008111),
                (48.4330972, -124.7000333),
                (48.3918250, -124.5030306),
            ],
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            True,
        ),
        (
            [
                (48.5850111, -125.0777611),
                (48.5764083, -124.8935528),
                (48.5229639, -124.6324806),
                (48.4163889, -124.5245667),
            ],
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            False,
        ),
        (
            [
                (48.3784667, -124.4983750),
                (48.4350583, -124.7806583),
                (48.3654278, -124.8312361),
                (48.2248778, -124.8450306),
            ],
            [
                (48.3864528, -125.0386028),
                (48.3601083, -124.8261639),
                (48.5799222, -124.8862583),
                (48.5148611, -124.6237444),
            ],
            2.0,
            True,
        ),
    ],
)
def test_LocalPath_global_path_changed(
    global_path_points, waypoints, buffer_distance, expected_result
):
    global_path = Path()
    for lat, lon in global_path_points:
        latlon = HelperLatLon(latitude=lat, longitude=lon)
        global_path.waypoints.append(latlon)

    # Create LocalPath instance with the test logger
    path_instance = local_path.LocalPath(parent_logger=test_logger)
    path_instance.waypoints = waypoints

    result = path_instance.global_path_changed(
        global_path=global_path, waypoints=waypoints, buffer=buffer_distance
    )
    assert (
        result == expected_result
    ), f"Failed for global path {global_path_points}, expected {expected_result} but got {result}"
