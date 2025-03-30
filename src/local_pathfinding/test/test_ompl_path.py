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
from ompl import base
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import Point

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState

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
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    ),
)


def test_OMPLPath___init__():
    assert OMPL_PATH.solved


def test_OMPLPath_get_cost():
    with pytest.raises(NotImplementedError):
        OMPL_PATH.get_cost()


def test_OMPLPath_get_waypoint():
    waypoints = OMPL_PATH.get_waypoints()
    waypoint_XY = cs.XY(OMPL_PATH.state.position.latitude, OMPL_PATH.state.position.longitude)
    start_state_latlon = cs.xy_to_latlon(OMPL_PATH.state.reference_latlon, waypoint_XY)

    test_start = waypoints[0]
    test_goal = waypoints[-1]

    assert (test_start.latitude, test_start.longitude) == pytest.approx(
        (start_state_latlon.latitude, start_state_latlon.longitude), abs=1e-2
    ), "first waypoint should be start state"
    assert (test_goal.latitude, test_goal.longitude) == pytest.approx(
        (OMPL_PATH.state.reference_latlon.latitude, OMPL_PATH.state.reference_latlon.longitude),
        abs=1e-2,
    ), "last waypoint should be goal state"


def test_OMPLPath_update_objectives():
    with pytest.raises(NotImplementedError):
        OMPL_PATH.update_objectives()


def test_init_obstacles():
    local_path_state = LocalPathState(
        gps=GPS(lat_lon=HelperLatLon(latitude=49.29, longitude=-126.32)),
        ais_ships=AISShips(
            ships=[
                HelperAISShip(
                    id=1,
                    lat_lon=HelperLatLon(latitude=49.28, longitude=-126.31),
                    cog=HelperHeading(heading=30.0),
                    sog=HelperSpeed(speed=20.0),
                    width=HelperDimension(dimension=20.0),
                    length=HelperDimension(dimension=100.0),
                    rot=HelperROT(rot=0),
                ),
                HelperAISShip(
                    id=2,
                    lat_lon=HelperLatLon(latitude=49.30, longitude=-126.31),
                    cog=HelperHeading(heading=60.0),
                    sog=HelperSpeed(speed=20.0),
                    width=HelperDimension(dimension=20.0),
                    length=HelperDimension(dimension=100.0),
                    rot=HelperROT(rot=0),
                ),
            ]
        ),
        global_path=Path(
            waypoints=[
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=1.0, longitude=1.0),
            ]
        ),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    )

    obstacles = ompl_path.OMPLPath.init_obstacles(local_path_state=local_path_state)
    assert isinstance(obstacles, list)
    assert isinstance(obstacles[0], ob.Boat)
    assert isinstance(obstacles[1], ob.Boat)
    assert isinstance(obstacles[2], ob.Land)


@pytest.mark.parametrize(
    "x,y,is_valid",
    [(0.5, 0.5, True), (-14, 0.5, False), (-16, 0.5, True)],
)
def test_is_state_valid(x: float, y: float, is_valid: bool):
    state = base.State(OMPL_PATH._simple_setup.getStateSpace())
    state().setXY(x, y)

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


@pytest.mark.parametrize(
    "position,expected_area,expected_bounds",
    [
        (cs.XY(0.0, 0.0), pytest.approx(4, rel=1e-2), (-1, -1, 1, 1)),
        (cs.XY(100.0, 100.0), pytest.approx(4, rel=1e-2), (99, 99, 101, 101)),
        (cs.XY(-100.0, -100.0), pytest.approx(4, rel=1e-2), (-101, -101, -99, -99)),
    ],
)
def test_create_space(position: cs.XY, expected_area, expected_bounds):
    """Test creation of buffered space around positions"""
    # Given an OMPLPath instance
    space = OMPL_PATH.create_buffer_around_position(position)

    assert space.area == expected_area, "Space area should match buffer size"
    assert space.bounds == pytest.approx(expected_bounds, abs=1.0), "Bounds should match expected"
    assert space.contains(Point(position.x, position.y)), "Space should contain center point"
