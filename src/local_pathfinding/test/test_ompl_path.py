import pyompl
import pytest
from custom_interfaces.msg import (
    GPS,
    AISShips,
    HelperAISShip,
    HelperDimension,
    HelperHeading,
    HelperLatLon,
    HelperROT,
    HelperSpeed,
    Path,
    WindSensor,
)
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
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


def test_OMPLPathState():
    local_path_state = LocalPathState(
        gps=GPS(),
        ais_ships=AISShips(),
        global_path=Path(),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    )
    state = ompl_path.OMPLPathState(local_path_state, logger=RcutilsLogger())
    assert state.state_domain == (-1, 1), "incorrect value for attribute state_domain"
    assert state.state_range == (-1, 1), "incorrect value for attribute start_state"
    assert state.start_state == pytest.approx(
        (0.5, 0.4)
    ), "incorrect value for attribute start_state"
    assert state.goal_state == pytest.approx(
        (0.5, -0.4)
    ), "incorrect value for attribute goal_state"


def test_OMPLPath___init__():
    assert PATH.solved


def test_OMPLPath_get_cost():
    with pytest.raises(NotImplementedError):
        PATH.get_cost()


def test_OMPLPath_get_waypoint():
    waypoints = PATH.get_waypoints()

    waypoint_XY = cs.XY(*PATH.state.start_state)
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
    [(0.5, 0.5, True), (-14, 0.5, False), (-16, 0.5, True)],
)
def test_is_state_valid(x: float, y: float, is_valid: bool):
    state = pyompl.ScopedState(PATH._simple_setup.getStateSpace())
    state.setXY(x, y)

    # Sample AIS SHIP message
    ais_ship = HelperAISShip(
        id=1,
        lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
        cog=HelperHeading(heading=0.0),
        sog=HelperSpeed(speed=18.52),
        width=HelperDimension(dimension=20.0),
        length=HelperDimension(dimension=100.0),
        rot=HelperROT(rot=0),
    )

    """A boat with these specific parameters has been visually verified to
    correspond correctly to the parameterized valid and non valid states for this test
    The visual verification is in this notebook:
    /workspaces/sailbot_workspace/src/local_pathfinding/land/land_polygons_notebook.ipynb
    """

    # Create a boat object
    boat1 = ob.Boat(
        HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
        HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
        30.0,
        ais_ship,
    )

    ompl_path.OMPLPath.obstacles.append(boat1)

    if is_valid:
        assert ompl_path.OMPLPath.is_state_valid(state), "state should be valid"
    else:
        assert not ompl_path.OMPLPath.is_state_valid(state), "state should not be valid"
