import math

import pytest
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as coord_systems
import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState
from local_pathfinding.objectives import get_true_wind_direction

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0

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
        global_waypoint_latlon=HelperLatLon(latitude=1.0, longitude=1.0),
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
        ((0, 0), (0, 0), 0.0, 0 * UPWIND_MULTIPLIER),
        ((-1, -1), (2, 1), 45.0, 3.605551275 * UPWIND_MULTIPLIER),
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
    "wind_direction_deg,heading_deg,expected",
    [
        (0, 0.0, True),
        (0.0, 45.0, False),
    ],
)
def test_is_upwind(wind_direction_deg: float, heading_deg: float, expected: float):
    wind_direction = math.radians(wind_direction_deg)
    heading = math.radians(heading_deg)

    assert objectives.WindObjective.is_upwind(wind_direction, heading) == expected


@pytest.mark.parametrize(
    "wind_direction_deg,heading_deg,expected",
    [
        (0.0, 0.0, False),
        (25.0, 46.0, False),
        (0, 180, True),
        (225, 45, True),
    ],
)
def test_is_downwind(wind_direction_deg: float, heading_deg: float, expected: float):
    wind_direction = math.radians(wind_direction_deg)
    heading = math.radians(heading_deg)

    assert objectives.WindObjective.is_downwind(wind_direction, heading) == expected


@pytest.mark.parametrize(
    "afir,amid,asec,expected",
    [
        (0, 1, 2, 1),
        (0, 20, 360, 0),
        (-20, 10, 40, 1),
        (0, 30, 60, 1),
        (-170, -130, -90, 1),
        (-170, -130, 100, 0),
        (400, 410, 420, 1),
        (400, 420, 410, 0),
        (370, 0, -370, 1),
        (370, 15, -370, 0),
        (-90, 270, 450, 0),
    ],
)
def test_angle_between(afir: float, amid: float, asec: float, expected: float):
    assert (
        objectives.WindObjective.is_angle_between(
            math.radians(afir), math.radians(amid), math.radians(asec)
        )
        == expected
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
    "wind_direction_degrees,wind_speed,heading_degrees,speed,expected_direction",
    [
        (0, 0, 0, 0, 0),
        (10, 0, 10, 10, 10),
        (179, 17, 179, 9, 179),
        (180, 17, 179, 9, 179.65),
        (140, 17, 45, 9, 111.06),
        (80, 5, -70, 8, -35.74),
    ],
)
def test_get_true_wind_direction(
    wind_direction_degrees: float,
    wind_speed: float,
    heading_degrees: float,
    speed: float,
    expected_direction: float,
):
    true_wind_direction = get_true_wind_direction(
        wind_direction_degrees, wind_speed, heading_degrees, speed
    )

    # Convert radians to degrees for easier comparison
    true_wind_direction_degrees = math.degrees(true_wind_direction)

    assert true_wind_direction_degrees == pytest.approx(expected=expected_direction, abs=1e-2)
