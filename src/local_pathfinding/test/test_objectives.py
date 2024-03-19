import itertools
import math

import pytest
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as coord_systems
import local_pathfinding.objectives as objectives
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState

# Upwind downwind cost multipliers
UPWIND_MULTIPLIER = 3000.0
DOWNWIND_MULTIPLIER = 3000.0


PATH = ompl_path.OMPLPath(
    parent_logger=RcutilsLogger(),
    max_runtime=1,
    local_path_state=LocalPathState(
        gps=GPS(),
        ais_ships=AISShips(),
        global_path=Path(),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    ),
)


""" Tests for distance objective """


@pytest.mark.parametrize(
    "method,max_motion_cost",
    [
        (objectives.DistanceMethod.EUCLIDEAN, 2.5),
        (objectives.DistanceMethod.LATLON, 2700),
        (objectives.DistanceMethod.OMPL_PATH_LENGTH, 4.0),
    ],
)
def test_distance_objective(method: objectives.DistanceMethod, max_motion_cost: float):
    distance_objective = objectives.DistanceObjective(
        PATH._simple_setup.getSpaceInformation(),
        method,
    )

    # test sample_states()
    num_samples = 3
    sampled_states = distance_objective.sample_states(num_samples)
    assert len(sampled_states) == num_samples

    # test find_maximum_motion_cost()
    assert distance_objective.max_motion_cost == pytest.approx(max_motion_cost, rel=1e0)

    # test if the motionCost() is normalized between 0 and 1 for 10 random samples
    states = distance_objective.sample_states(10)
    for s1, s2 in itertools.combinations(iterable=states, r=2):
        assert 0 <= distance_objective.motionCost(s1, s2).value() <= 1


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
    "rf, cs1,cs2",
    [
        ((10.0, 10.0), (0.0, 0.0), (0.0, 0.0)),
        ((13.206724, 29.829011), (13.208724, 29.827011), (13.216724, 29.839011)),
        ((0.0, 0.0), (0.0, 0.1), (0.0, -0.1)),
        ((0.0, 0.0), (0.1, 0.0), (-0.1, 0.0)),
        ((0.0, 0.0), (0.1, 0.1), (-0.1, -0.1)),
    ],
)
def test_get_latlon_path_length_objective(rf: tuple, cs1: tuple, cs2: tuple):
    reference = HelperLatLon(latitude=rf[0], longitude=rf[1])
    s1 = HelperLatLon(latitude=cs1[0], longitude=cs1[1])
    s2 = HelperLatLon(latitude=cs2[0], longitude=cs2[1])
    ls1 = coord_systems.latlon_to_xy(reference, s1)
    ls2 = coord_systems.latlon_to_xy(reference, s2)
    _, _, distance_m = coord_systems.GEODESIC.inv(
        lats1=s1.latitude,
        lons1=s1.longitude,
        lats2=s2.latitude,
        lons2=s2.longitude,
    )

    assert objectives.DistanceObjective.get_latlon_path_length_objective(
        ls1,
        ls2,
        reference,
    ) == pytest.approx(distance_m)


""" Tests for minimum turning objective """


@pytest.mark.parametrize(
    "method,heading_degrees,max_motion_cost",
    [
        (objectives.MinimumTurningMethod.GOAL_HEADING, 60.0, 174.862),
        (objectives.MinimumTurningMethod.GOAL_PATH, 60.0, 179.340),
        (objectives.MinimumTurningMethod.HEADING_PATH, 60.0, 179.962),
    ],
)
def test_minimum_turning_objective(
    method: objectives.MinimumTurningMethod, heading_degrees: float, max_motion_cost: float
):
    minimum_turning_objective = objectives.MinimumTurningObjective(
        PATH._simple_setup.getSpaceInformation(),
        PATH._simple_setup,
        heading_degrees,
        method,
    )

    # test sample_states()
    num_samples = 3
    sampled_states = minimum_turning_objective.sample_states(num_samples)
    assert len(sampled_states) == num_samples

    # test find_maximum_motion_cost()
    assert minimum_turning_objective.max_motion_cost == pytest.approx(max_motion_cost, rel=1e0)

    # test if the motionCost() is normalized between 0 and 1 for 10 random samples
    states = minimum_turning_objective.sample_states(10)
    for s1, s2 in itertools.combinations(iterable=states, r=2):
        assert 0 <= minimum_turning_objective.motionCost(s1, s2).value() <= 1


@pytest.mark.parametrize(
    "cs1,sf,heading_degrees,expected",
    [
        ((0, 0), (0, 0), 0, 0),
        ((-1, -1), (0.1, 0.2), 45, 2.490),
    ],
)
def test_goal_heading_turn_cost(cs1: tuple, sf: tuple, heading_degrees: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    goal = coord_systems.XY(*sf)
    heading = math.radians(heading_degrees)
    assert objectives.MinimumTurningObjective.goal_heading_turn_cost(
        s1, goal, heading
    ) == pytest.approx(expected, abs=1e-3)


@pytest.mark.parametrize(
    "cs1,cs2,sf,expected",
    [
        ((0, 0), (0, 0), (0, 0), 0),
        ((-1, -1), (2, 1), (0.1, 0.2), 13.799),
    ],
)
def test_goal_path_turn_cost(cs1: tuple, cs2: tuple, sf: tuple, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    goal = coord_systems.XY(*sf)

    assert objectives.MinimumTurningObjective.goal_path_turn_cost(s1, s2, goal) == pytest.approx(
        expected, abs=1e-3
    )


@pytest.mark.parametrize(
    "cs1,cs2,heading_degrees,expected",
    [
        ((0, 0), (0, 0), 0.0, 0),
        ((-1, -1), (2, 1), 45.0, 11.310),
    ],
)
def test_heading_path_turn_cost(cs1: tuple, cs2: tuple, heading_degrees: float, expected: float):
    s1 = coord_systems.XY(*cs1)
    s2 = coord_systems.XY(*cs2)
    heading = math.radians(heading_degrees)

    assert objectives.MinimumTurningObjective.heading_path_turn_cost(
        s1, s2, heading
    ) == pytest.approx(expected, abs=1e-3)


""" Tests for wind objective """


@pytest.mark.parametrize(
    "wind_direction_deg,max_motion_cost",
    [
        (60.0, 7593.768),
        (45.0, 7763.842),
        (0.0, 7579.767),
    ],
)
def test_wind_objective(wind_direction_deg: float, max_motion_cost: float):
    wind_objective = objectives.WindObjective(
        PATH._simple_setup.getSpaceInformation(), wind_direction_deg
    )

    # test sample_states()
    num_samples = 3
    sampled_states = wind_objective.sample_states(num_samples)
    assert len(sampled_states) == num_samples

    # test find_maximum_motion_cost()
    assert wind_objective.max_motion_cost == pytest.approx(max_motion_cost, rel=1e0)

    # test if the motionCost() is normalized between 0 and 1 for 10 random samples
    states = wind_objective.sample_states(10)
    for s1, s2 in itertools.combinations(iterable=states, r=2):
        assert 0 <= wind_objective.motionCost(s1, s2).value() <= 1


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


""" Tests for speed objective """


@pytest.mark.parametrize(
    "method,heading,wind_direction,wind_speed,max_motion_cost",
    [
        (objectives.SpeedObjectiveMethod.SAILBOT_TIME, 5, 43, 28, 3672.594),
        (objectives.SpeedObjectiveMethod.SAILBOT_PIECEWISE, -15, 67, 22, 50.0),
        (objectives.SpeedObjectiveMethod.SAILBOT_CONTINUOUS, -15, 67, 22, 10000.0),
    ],
)
def test_speed_objective(
    method: objectives.SpeedObjectiveMethod,
    heading,
    wind_direction,
    wind_speed,
    max_motion_cost: float,
):
    speed_objective = objectives.SpeedObjective(
        PATH._simple_setup.getSpaceInformation(),
        heading,
        wind_direction,
        wind_speed,
        method,
    )

    # test sample_states()
    num_samples = 3
    sampled_states = speed_objective.sample_states(num_samples)
    assert len(sampled_states) == num_samples

    # test find_maximum_motion_cost()
    if method != objectives.SpeedObjectiveMethod.SAILBOT_TIME:
        assert speed_objective.max_motion_cost == pytest.approx(max_motion_cost, rel=1e0)

    # test if the motionCost() is normalized between 0 and 1 for 10 random samples
    states = speed_objective.sample_states(10)
    for s1, s2 in itertools.combinations(iterable=states, r=2):
        assert 0 <= speed_objective.motionCost(s1, s2).value() <= 1


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
    "speed,expected",
    [
        (0.0, 5),
        (8, 10),
        (12.5, 20),
        (17.0, 50),
        (35, 10000),
    ],
)
def test_piecewise_cost(speed: float, expected: int):
    assert objectives.SpeedObjective.get_piecewise_cost(speed) == expected


@pytest.mark.parametrize(
    "speed,expected",
    [
        (0.0, 10000),
        (25.0, 10000),
        (30, 2.2013016167),
        (40, 1.55146222424),
        (10, 0.551462224238),
    ],
)
def test_continuous_cost(speed: float, expected: int):
    assert objectives.SpeedObjective.get_continuous_cost(speed) == pytest.approx(
        expected, abs=1e-3
    )
