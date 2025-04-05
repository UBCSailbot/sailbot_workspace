import custom_interfaces.msg as ci
import pytest

import local_pathfinding.node_navigate as nn


@pytest.mark.parametrize(
    "local_path, waypoint_index, boat_lat_lon, correct_heading, new_wp_index",
    [
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=-0.1),
            ci.HelperHeading(heading=90.0),
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.0),
            ci.HelperHeading(heading=-180.0),
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.1),
            ci.HelperHeading(heading=-135.0),
            0,
        ),
        (
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.1),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=0.09999),
            ci.HelperHeading(heading=-90.0),
            0,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index(
    path: ci.Path,
    waypoint_index: int,
    boat_lat_lon: ci.HelperLatLon,
    correct_heading: ci.HelperHeading,
    new_wp_index: int,
):
    calculated_answer = nn.Sailbot.calculate_desired_heading_and_waypoint_index(
        path, waypoint_index, boat_lat_lon
    )
    assert calculated_answer[0] == pytest.approx(correct_heading)
    assert calculated_answer[1] == pytest.approx(new_wp_index)
