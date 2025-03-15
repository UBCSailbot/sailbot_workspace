import math
from unittest.mock import MagicMock

import pytest
import rclpy.node
from custom_interfaces.msg import (
    GPS,
    AISShips,
    DesiredHeading,
    HelperLatLon,
    Path,
    WindSensor,
)
from pyproj import Geod

from local_pathfinding.local_path import LocalPath
from local_pathfinding.node_navigate import Sailbot

# # def fake_node_init(self, node_name, *args, **kwargs):
# #     self.node_name = node_name
# #     self.get_logger = lambda: MagicMock()
# # rclpy.node.Node.__init__ = fake_node_init

GEODESIC = Geod(ellps="WGS84")


def normalize_angle(angle: float) -> float:
    return angle if angle >= 0 else angle + 360


# @pytest.fixture

# def test_get_desired_heading_no_subscribers():
#     sailbot.ais_ships = None
#     sailbot.gps = None
#     sailbot.global_path = None
#     sailbot.filtered_wind_sensor = None
#     assert sailbot.get_desired_heading() == -1.0

# def test_get_desired_heading_all_subscribers_active():
#     sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
#     sailbot.global_path = Path(waypoints=[HelperLatLon(latitude=49.0, longitude=-123.0)])
#     sailbot.ais_ships = AISShips()
#     sailbot.filtered_wind_sensor = WindSensor()
#     heading = sailbot.get_desired_heading()
#     assert 0 <= heading < 360

# def test_get_desired_heading_at_final_waypoint():
#     sailbot.gps = GPS(position=HelperLatLon(latitude=50.0, longitude=-124.0))
#     sailbot.local_path.waypoints = [HelperLatLon(latitude=50.0, longitude=-124.0)]
#     sailbot.current_waypoint_index = len(sailbot.local_path.waypoints) - 1
#     assert sailbot.get_desired_heading() == -1.0

# @pytest.mark.parametrize(
#     "boat_lat, boat_long, current_lat, current_long, expected_heading",
#     [
#         (48.5, -123.4, 48.5, -123.4, 0.0),
#         (48.5, -123.4, 49.0, -123.0, 45.0),
#         (48.5, -123.4, 48.5, -122.0, 90.0),
#         (48.5, -123.4, 47.0, -123.4, 180.0),
#         (48.5, -123.4, 48.5, -124.5, 270.0),
#     ],
# )
# def test_get_angle(boat_lat, boat_long, current_lat, current_long, expected_heading):
#     heading = sailbot.get_angle(boat_lat, boat_long, current_lat, current_long)
#     heading_norm = normalize_angle(heading)
#     expected_norm = normalize_angle(expected_heading)
#     assert 0 <= heading_norm < 360
#     assert math.isclose(heading_norm, expected_norm, abs_tol=5.0)

# @pytest.mark.parametrize(
#     "distance, threshold, current_index, expected_index",
#     [
#         (0.4, 0.5, 0, 1),
#         (0.6, 0.5, 0, 0),
#         (0.0, 0.5, 1, 2),
#     ],
# )
# def test_update_waypoint(distance, threshold, current_index, expected_index):
#     sailbot.current_waypoint_index = current_index
#     sailbot.local_path.waypoints = [HelperLatLon(48.5, -123.4)] * 5
#     sailbot.update_waypoint(distance, threshold)
#     assert sailbot.current_waypoint_index == expected_index

# @pytest.mark.parametrize(
#     "lat1, lon1, lat2, lon2, expected_distance",
#     [
#         (48.0, -123.0, 48.0, -123.0, 0.0),
#         (48.0, -123.0, 49.0, -123.0, 111000),
#     ],
# )
# def test_calculate_distance(lat1, lon1, lat2, lon2, expected_distance):
#     distance = Sailbot.calculate_distance(lat1, lon1, lat2, lon2)
#     assert math.isclose(distance, expected_distance, rel_tol=0.05)


@pytest.mark.parametrize(
    "lat1, lon1, lat2, lon2, expected_angle",
    [
        (48.0, -123.0, 49.0, -123.0, 0.0),
        (48.0, -123.0, 48.0, -122.0, 90.0),
        (49.0, -123.0, 48.0, -123.0, 180.0),
        (48.0, -123.0, 48.0, -124.0, 270.0),
    ],
)
def test_calculate_angle(lat1, lon1, lat2, lon2, expected_angle):
    angle = Sailbot.calculate_angle(lat1, lon1, lat2, lon2)
    angle_norm = normalize_angle(angle)
    expected_norm = normalize_angle(expected_angle)
    assert math.isclose(angle_norm, expected_norm, abs_tol=5.0)


# def test_get_desired_heading_invalid_gps():
#     sailbot.gps = GPS(position=HelperLatLon(latitude=None, longitude=None))
#     assert sailbot.get_desired_heading() == -1.0

# def test_get_desired_heading_large_waypoints():
#     sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
#     sailbot.local_path.waypoints = [HelperLatLon(latitude=i, longitude=i) for i in range(1000)]
#     heading = sailbot.get_desired_heading()
#     assert 0 <= heading < 360

# def test_get_desired_heading_looped_path():
#     sailbot.gps = GPS(position=HelperLatLon(latitude=48.5, longitude=-123.4))
#     sailbot.local_path.waypoints = [
#         HelperLatLon(latitude=48.5, longitude=-123.4),
#         HelperLatLon(latitude=49.0, longitude=-123.0),
#         HelperLatLon(latitude=48.5, longitude=-123.4),
#     ]
#     heading = sailbot.get_desired_heading()
#     assert 0 <= heading < 360
