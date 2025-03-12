import pytest
import math
from unittest.mock import MagicMock
from custom_interfaces.msg import GPS, AISShips, Path, WindSensor, DesiredHeading, HelperLatLon
from local_pathfinding.local_path import LocalPath
from local_pathfinding.sailbot import Sailbot
from pyproj import Geod


GEODESIC = Geod(ellps="WGS84")


@pytest.fixture
def sailbot():
    """Fixture to create a Sailbot instance for testing."""
    bot = Sailbot()
    bot.get_logger = MagicMock()  # Mock the logger
    return bot


# ------------------ TEST get_desired_heading ------------------
def test_get_desired_heading_no_subscribers(sailbot):
    sailbot.ais_ships = None
    sailbot.gps = None
    sailbot.global_path = None
    sailbot.filtered_wind_sensor = None
    assert sailbot.get_desired_heading() == -1.0, "Expected -1.0 for inactive subscribers"


def test_get_desired_heading_all_subscribers_active(sailbot):
    sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
    sailbot.global_path = Path(waypoints=[HelperLatLon(latitude=49.0, longitude=-123.0)])
    sailbot.ais_ships = AISShips()
    sailbot.filtered_wind_sensor = WindSensor()

    heading = sailbot.get_desired_heading()
    assert 0 <= heading < 360, "Expected heading to be in range [0, 360)"


def test_get_desired_heading_at_final_waypoint(sailbot):
    sailbot.gps = GPS(position=HelperLatLon(latitude=50.0, longitude=-124.0))
    sailbot.local_path.waypoints = [HelperLatLon(latitude=50.0, longitude=-124.0)]
    sailbot.current_waypoint_index = len(sailbot.local_path.waypoints) - 1

    assert sailbot.get_desired_heading() == -1.0, "Expected -1.0 at final waypoint"


# ------------------ TEST get_angle ------------------
@pytest.mark.parametrize(
    "boat_lat, boat_long, current_lat, current_long, expected_heading",
    [
        (48.5, -123.4, 48.5, -123.4, 0.0),  # Same location
        (48.5, -123.4, 49.0, -123.0, 45.0),  # NE direction
        (48.5, -123.4, 48.5, -122.0, 90.0),  # East
        (48.5, -123.4, 47.0, -123.4, 180.0),  # South
        (48.5, -123.4, 48.5, -124.5, 270.0),  # West
    ],
)
def test_get_angle(sailbot, boat_lat, boat_long, current_lat, current_long, expected_heading):
    heading = sailbot.get_angle(boat_lat, boat_long, current_lat, current_long)
    assert 0 <= heading < 360, "Heading should be within [0, 360)"
    assert math.isclose(heading, expected_heading, abs_tol=5.0), "Unexpected heading angle"


# ------------------ TEST update_waypoint ------------------
@pytest.mark.parametrize(
    "distance, threshold, current_index, expected_index",
    [
        (0.4, 0.5, 0, 1),  # Close enough to update
        (0.6, 0.5, 0, 0),  # Not close enough, should not update
        (0.0, 0.5, 1, 2),  # At the waypoint, should update
    ],
)
def test_update_waypoint(sailbot, distance, threshold, current_index, expected_index):
    sailbot.current_waypoint_index = current_index
    sailbot.local_path.waypoints = [HelperLatLon(48.5, -123.4)] * 5  # Mock waypoints
    sailbot.update_waypoint(distance, threshold)

    assert sailbot.current_waypoint_index == expected_index, "Waypoint index did not update correctly"


# ------------------ TEST calculate_distance ------------------
@pytest.mark.parametrize(
    "lat1, lon1, lat2, lon2, expected_distance",
    [
        (48.0, -123.0, 48.0, -123.0, 0.0),  # Same point
        (48.0, -123.0, 49.0, -123.0, 111000),  # Approximate 1-degree latitude difference
    ],
)
def test_calculate_distance(lat1, lon1, lat2, lon2, expected_distance):
    distance = Sailbot.calculate_distance(lat1, lon1, lat2, lon2)
    assert math.isclose(distance, expected_distance, rel_tol=0.05), "Incorrect distance calculation"


# ------------------ TEST calculate_angle ------------------
@pytest.mark.parametrize(
    "lat1, lon1, lat2, lon2, expected_angle",
    [
        (48.0, -123.0, 49.0, -123.0, 0.0),  # North
        (48.0, -123.0, 48.0, -122.0, 90.0),  # East
        (49.0, -123.0, 48.0, -123.0, 180.0),  # South
        (48.0, -123.0, 48.0, -124.0, 270.0),  # West
    ],
)
def test_calculate_angle(lat1, lon1, lat2, lon2, expected_angle):
    angle = Sailbot.calculate_angle(lat1, lon1, lat2, lon2)
    assert math.isclose(angle, expected_angle, abs_tol=5.0), "Incorrect angle calculation"


# ------------------ Additional Edge Cases ------------------
def test_get_desired_heading_invalid_gps(sailbot):
    sailbot.gps = GPS(position=HelperLatLon(latitude=None, longitude=None))
    assert sailbot.get_desired_heading() == -1.0, "Expected -1.0 for invalid GPS data"


def test_get_desired_heading_large_waypoints(sailbot):
    sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
    sailbot.local_path.waypoints = [HelperLatLon(latitude=i, longitude=i) for i in range(1000)]

    heading = sailbot.get_desired_heading()
    assert 0 <= heading < 360, "Expected heading to be in range [0, 360) with large waypoints"


def test_get_desired_heading_looped_path(sailbot):
    sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
    sailbot.local_path.waypoints = [
        HelperLatLon(latitude=48.5, longitude=-123.4),
        HelperLatLon(latitude=49.0, longitude=-123.0),
        HelperLatLon(latitude=48.5, longitude=-123.4),
    ]

    heading = sailbot.get_desired_heading()
    assert 0 <= heading < 360, "Expected heading to be valid even in a looped path"
