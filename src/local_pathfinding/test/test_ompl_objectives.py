import math

import pytest
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import Point

import local_pathfinding.coord_systems as coord_systems
import local_pathfinding.ompl_objectives as objectives
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState
from local_pathfinding.ompl_objectives import (
    create_buffer_around_position,
    get_true_wind,
)

OMPL_PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=LocalPathState(
        gps=GPS(),
        ais_ships=AISShips(),
        global_path=Path(
            waypoints=[
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=1.0, longitude=1.0),
            ]
        ),
        target_global_waypoint=HelperLatLon(latitude=1.0, longitude=1.0),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    ),
)


@pytest.mark.parametrize(
    "cs1,cs2,expected",
    [
        ((0, 0), (0, 0), 0),
        ((0.5, 0.5), (0.1, 0.2), 0.5),
    ],
)
def test_get_euclidean_path_length_objective(cs1: tuple, cs2: tuple, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    assert objectives.DistanceObjective.get_euclidean_path_length_objective(s1, s2) == expected


@pytest.mark.parametrize(
    "cs1,cs2,wind_direction_deg,expected",
    [
        # Moving directly into wind (upwind)
        ((0, 0), (0, 1), 0.0, objectives.UPWIND_MULTIPLIER * 1.0),
        # Moving perpendicular to wind (crosswind)
        ((0, 0), (1, 0), 0.0, 0.0),
        # Moving directly with the wind (downwind)
        ((0, 0), (0, -1), 0.0, objectives.DOWNWIND_MULTIPLIER * 1.0),
        # Moving at 45° off upwind
        ((0, 0), (1, 1), 0.0, objectives.UPWIND_MULTIPLIER * math.cos(math.radians(45))),
        # Moving 30° off upwind
        (
            (0, 0),
            (math.sin(math.radians(30)), math.cos(math.radians(30))),
            0.0,
            objectives.UPWIND_MULTIPLIER * math.cos(math.radians(30)),
        ),
        # Moving 135° off upwind (45° off downwind)
        (
            (0, 0),
            (math.sin(math.radians(135)), math.cos(math.radians(135))),
            0.0,
            objectives.DOWNWIND_MULTIPLIER * abs(math.cos(math.radians(135))),
        ),
        # Wind from 179°, boat moving North
        ((0, 0), (0, 1), 179.0, objectives.DOWNWIND_MULTIPLIER * math.cos(math.radians(1))),
        # Wind from -179°, boat moving North
        ((0, 0), (0, 1), -179.0, objectives.DOWNWIND_MULTIPLIER * math.cos(math.radians(1))),
    ],
)
def test_wind_direction_cost(cs1: tuple, cs2: tuple, wind_direction_deg: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    wind_direction = math.radians(wind_direction_deg)
    assert objectives.WindObjective.wind_direction_cost(s1, s2, wind_direction) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "heading,wind_direction,wind_speed,expected",
    [
        # Corners of the table
        (0, 0, 0, 0),
        (-90, 90, 37.0, 18.5),
        (0, 180, 0, 0),
        (0, 0, 37.0, 0),
        # Edges of table
        (-48, 22, 0, 0),
        (-22, 140, 0, 0),
        (63, 63, 9.3, 0),
        (-81, -81, 32.3, 0),
        # Other edge cases
        (60, -120, 10.6, 3.704347826),
        (170, -155, 37, 6.833333333),
        (-50, -152.7, 27.8, 15.844222222),
        (-170, 160, 14.4, 1.231521739),
        (0, 45, 18.5, 3.7),
        # General cases
        (-20, 40, 12.0, 2.905434783),
        (12.9, -1, 5.3, 0),
    ],
)
def test_get_sailbot_speed(
    heading: float, wind_direction: float, wind_speed: float, expected: float
):
    assert objectives.SpeedObjective.get_sailbot_speed(
        heading, wind_direction, wind_speed
    ) == pytest.approx(expected, abs=1e-7)


@pytest.mark.parametrize(
    "wind_direction_degrees,wind_speed,heading_degrees,speed,expected_direction, expected_speed",
    [
        (0, 0, 0, 0, 0, 0),
        (10, 0, 10, 10, 10, 10),
        (179, 17, 179, 9, 179, 26),
        (180, 17, 179, 9, 179.65, 26),
        (140, 17, 45, 9, 111.06, 18.52),
        (80, 5, -70, 8, -35.74, 4.44),
    ],
)
def test_get_true_wind_direction(
    wind_direction_degrees: float,
    wind_speed: float,
    heading_degrees: float,
    speed: float,
    expected_direction: float,
    expected_speed: float,
):
    true_wind_direction, true_wind_speed = get_true_wind(
        wind_direction_degrees, wind_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    true_wind_direction_degrees = math.degrees(true_wind_direction)

    assert true_wind_direction_degrees == pytest.approx(
        expected=expected_direction, abs=1e-2
    ) and true_wind_speed == pytest.approx(expected=expected_speed, abs=1e-2)


@pytest.mark.parametrize(
    "position,expected_area,expected_bounds,box_buffer_size",
    [
        (coord_systems.XY(0.0, 0.0), pytest.approx(4, rel=1e-2), (-1, -1, 1, 1), 1.0),
        (coord_systems.XY(100.0, 100.0), pytest.approx(4, rel=1e-2), (99, 99, 101, 101), 1.0),
        (coord_systems.XY(-50.0, -50.0), pytest.approx(4, rel=1e-2), (-51, -51, -49, -49), 1.0),
        (coord_systems.XY(100.0, 100.0), pytest.approx(36, rel=1e-2), (97, 97, 103, 103), 3.0),
        (coord_systems.XY(-50.0, -50.0), pytest.approx(36, rel=1e-2), (-53, -53, -47, -47), 3.0),
    ],
)
def test_create_space(
    position: coord_systems.XY, expected_area, expected_bounds, box_buffer_size: float
):
    """Test creation of buffered space around positions"""
    # Given an OMPLPath instance

    space = create_buffer_around_position(position, box_buffer_size)

    assert space.area == expected_area, "Space area should match buffer size"
    assert space.bounds == pytest.approx(
        expected_bounds, abs=box_buffer_size
    ), "Bounds should match expected"
    assert space.contains(Point(position.x, position.y)), "Space should contain center point"
