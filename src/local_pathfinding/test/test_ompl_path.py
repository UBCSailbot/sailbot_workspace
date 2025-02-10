import pyompl
import pytest
from custom_interfaces.msg import GPS, AISShips, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as cs
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState

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

# TODO: Update this test
# def test_OMPLPathState():
#     local_path_state = LocalPathState(
#         gps=GPS(),
#         ais_ships=AISShips(),
#         global_path=Path(),
#         filtered_wind_sensor=WindSensor(),
#         planner="rrtstar",
#     )
#     state = local_path_state
#     # assert state.state_domain == (-1, 1), "incorrect value for attribute state_domain"
#     # assert state.state_range == (-1, 1), "incorrect value for attribute start_state"
#     # assert state.start_state == pytest.approx(
#     #     (0.5, 0.4)
#     # ), "incorrect value for attribute start_state"
#     # assert state.goal_state == pytest.approx(
#     #     (0.5, -0.4)
#     # ), "incorrect value for attribute goal_state"


def test_OMPLPath___init__():
    assert PATH.solved


def test_OMPLPath_get_cost():
    with pytest.raises(NotImplementedError):
        PATH.get_cost()


def test_OMPLPath_get_waypoint():
    waypoints = PATH.get_waypoints()
    waypoint_XY = cs.XY(PATH.state.position[0], PATH.state.position[1])
    start_state_latlon = cs.xy_to_latlon(PATH.state.reference_latlon, waypoint_XY)

    test_start = waypoints[0]
    test_goal = waypoints[-1]

    assert (test_start.latitude, test_start.longitude) == pytest.approx(
        (start_state_latlon.latitude, start_state_latlon.longitude), abs=1e-2
    ), "first waypoint should be start state"
    assert (test_goal.latitude, test_goal.longitude) == pytest.approx(
        (PATH.state.reference_latlon.latitude, PATH.state.reference_latlon.longitude), abs=1e-2
    ), "last waypoint should be goal state"


def test_OMPLPath_update_objectives():
    with pytest.raises(NotImplementedError):
        PATH.update_objectives()


@pytest.mark.parametrize(
    "x,y,is_valid",
    [
        (0.5, 0.5, True),
        (0.6, 0.6, False),
    ],
)
def test_is_state_valid(x: float, y: float, is_valid: bool):
    state = pyompl.ScopedState(PATH._simple_setup.getStateSpace())
    state.setXY(x, y)

    if is_valid:
        assert ompl_path.is_state_valid(state), "state should be valid"
    else:
        assert not ompl_path.is_state_valid(state), "state should not be valid"
