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
