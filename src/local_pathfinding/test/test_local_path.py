from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.local_path as local_path

PATH = local_path.LocalPath(parent_logger=RcutilsLogger())


def test_LocalPath_update_if_needed():
    PATH.update_if_needed(
        gps=GPS(),
        ais_ships=AISShips(),
        global_path=Path(),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    )
    assert PATH.waypoints is not None, "waypoints is not initialized"
    assert len(PATH.waypoints) > 1, "waypoints length <= 1"


def test_LocalPath_sailbot_drifted_from_old_path():
    def create_gps(lat, lon):
        gps = GPS()
        gps.lat_lon = HelperLatLon(latitude=lat, longitude=lon)
        return gps

    # Define the waypoints of old_path
    waypoints = [
        (48.3864528, -125.0386028),
        (48.3601083, -124.8261639),
        (48.5799222, -124.8862583),
        (48.5148611, -124.6237444),
    ]
    path_instance = local_path.LocalPath(parent_logger=None)
    path_instance.waypoints = waypoints

    # Test GPS points manually extracted
    gps_points = [
        (48.3898917, -125.0318278, False),
        (48.3775278, -124.9368167, False),
        (48.4439528, -124.9530528, True),
        (48.5755778, -124.8806528, False),
        (48.5136611, -124.6219000, False),
    ]

    buffer_distance = 2.0  # km

    # Run the test for each GPS point
    for lat, lon, expected_result in gps_points:
        gps = create_gps(lat, lon)
        result = path_instance.sailbot_drifted_from_old_path(
            gps=gps, waypoints=waypoints, buffer=buffer_distance
        )
        assert (
            result == expected_result
        ), f"Test failed for GPS ({lat}, {lon}), expected {expected_result} but got {result}"


def test_LocalPath_global_path_changed():
    def create_global_path(points):
        path = Path()
        for lat, lon in points:
            latlon = HelperLatLon(latitude=lat, longitude=lon)
            path.waypoints.append(latlon)
        return path

    waypoints = [
        (48.3864528, -125.0386028),
        (48.3601083, -124.8261639),
        (48.5799222, -124.8862583),
        (48.5148611, -124.6237444),
    ]

    path_instance = local_path.LocalPath(parent_logger=None)
    path_instance.waypoints = waypoints

    global_paths = [
        (
            [
                (48.5487833, -125.1470583),
                (48.4804222, -125.0008111),
                (48.4330972, -124.7000333),
                (48.3918250, -124.5030306),
            ],
            True,
        ),
        (
            [
                (48.5850111, -125.0777611),
                (48.5764083, -124.8935528),
                (48.5229639, -124.6324806),
                (48.4163889, -124.5245667),
            ],
            False,
        ),
        (
            [
                (48.3784667, -124.4983750),
                (48.4350583, -124.7806583),
                (48.3654278, -124.8312361),
                (48.2248778, -124.8450306),
            ],
            True,
        ),
    ]

    buffer_distance = 2.0  # km

    # Run the test for each global path
    for path_points, expected_result in global_paths:
        global_path = create_global_path(path_points)
        result = path_instance.global_path_changed(
            global_path=global_path, waypoints=waypoints, buffer=buffer_distance
        )
        assert (
            result == expected_result
        ), f"Test failed for global path {path_points}, expected {expected_result} but got {result}"
