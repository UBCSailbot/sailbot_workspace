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
from shapely.geometry import MultiPolygon, Point, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
import local_pathfinding.ompl_path as ompl_path
from local_pathfinding.local_path import LocalPathState

LAND_KEY = -1


@pytest.fixture
def fresh_ompl_path():
    return ompl_path.OMPLPath(
        parent_logger=RcutilsLogger(),
        local_path_state=LocalPathState(
            gps=GPS(lat_lon=HelperLatLon(latitude=0.0, longitude=0.0)),
            ais_ships=AISShips(),
            global_path=Path(
                waypoints=[
                    HelperLatLon(latitude=0.02, longitude=0.02),
                    HelperLatLon(latitude=0.04, longitude=0.03),
                    HelperLatLon(latitude=0.06, longitude=0.05),
                    HelperLatLon(latitude=0.08, longitude=0.07),
                    HelperLatLon(latitude=0.10, longitude=0.09),
                    HelperLatLon(latitude=0.12, longitude=0.11),
                    HelperLatLon(latitude=0.15, longitude=0.15),
                    HelperLatLon(latitude=0.1, longitude=0.1),
                ]
            ),
            target_global_waypoint=HelperLatLon(latitude=0.1, longitude=0.1),
            filtered_wind_sensor=WindSensor(),
            planner="rrtstar",
        ),
    )


def test_OMPLPath___init__(fresh_ompl_path):
    assert fresh_ompl_path.solved


def test_OMPLPath_get_waypoint(fresh_ompl_path):
    waypoints = fresh_ompl_path.get_path().waypoints  # List[HelperLatLon]
    start_state_latlon = fresh_ompl_path.state.position

    test_start = waypoints[0]
    test_goal = waypoints[-1]

    assert (test_start.latitude, test_start.longitude) == pytest.approx(
        (start_state_latlon.latitude, start_state_latlon.longitude), abs=1e-2
    ), "first waypoint should be start state"
    assert (test_goal.latitude, test_goal.longitude) == pytest.approx(
        (
            fresh_ompl_path.state.reference_latlon.latitude,
            fresh_ompl_path.state.reference_latlon.longitude,
        ),
        abs=1e-2,
    ), "last waypoint should be goal state"


def test_OMPLPath_update_objectives(fresh_ompl_path):
    with pytest.raises(NotImplementedError):
        fresh_ompl_path.update_objectives()


def test_init_obstacles():
    sailbot_position = HelperLatLon(latitude=49.29, longitude=-126.32)
    goal_position = HelperLatLon(latitude=1.0, longitude=1.0)

    local_path_state = LocalPathState(
        gps=GPS(lat_lon=sailbot_position),
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
                goal_position,
            ]
        ),
        target_global_waypoint=goal_position,
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    )

    # create the xy state space from the specified positions of sailbot and the goal
    sailbot_box = Point(sailbot_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    goal_box = Point(goal_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    state_space_latlon = box(*MultiPolygon([sailbot_box, goal_box]).bounds)

    state_space_xy = cs.latlon_polygon_list_to_xy_polygon_list(
        [state_space_latlon], goal_position
    )[0]

    obstacles = ompl_path.OMPLPath.init_obstacles(
        local_path_state=local_path_state, state_space_xy=state_space_xy
    )

    old_boat1 = obstacles[1]

    assert isinstance(obstacles, dict)
    assert 1 in obstacles.keys()
    assert 2 in obstacles.keys()
    assert isinstance(obstacles[1], ob.Boat)
    assert isinstance(obstacles[2], ob.Boat)
    assert LAND_KEY in obstacles.keys()

    sailbot_position = HelperLatLon(latitude=49.29, longitude=-126.32)
    goal_position = HelperLatLon(latitude=1.0, longitude=1.0)
    sailbot_box = Point(sailbot_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    goal_box = Point(goal_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    state_space_latlon = box(*MultiPolygon([sailbot_box, goal_box]).bounds)

    state_space_xy = cs.latlon_polygon_list_to_xy_polygon_list(
        [state_space_latlon], goal_position
    )[0]

    # Call again with one existing boat (id=1) and one new (id=3), i.e. id=2 should be evicted
    updated_local_path_state = LocalPathState(
        gps=GPS(lat_lon=sailbot_position),
        ais_ships=AISShips(
            ships=[
                HelperAISShip(
                    id=1,
                    lat_lon=HelperLatLon(latitude=49.2801, longitude=-126.3101),
                    cog=HelperHeading(heading=35.0),
                    sog=HelperSpeed(speed=19.0),
                    width=HelperDimension(dimension=22.0),
                    length=HelperDimension(dimension=102.0),
                    rot=HelperROT(rot=1),
                ),
                # new boat
                HelperAISShip(
                    id=3,
                    lat_lon=HelperLatLon(latitude=49.31, longitude=-126.29),
                    cog=HelperHeading(heading=90.0),
                    sog=HelperSpeed(speed=15.0),
                    width=HelperDimension(dimension=18.0),
                    length=HelperDimension(dimension=80.0),
                    rot=HelperROT(rot=0),
                ),
            ]
        ),
        global_path=Path(
            waypoints=[
                HelperLatLon(latitude=0.0, longitude=0.0),
                goal_position,
            ]
        ),
        target_global_waypoint=HelperLatLon(latitude=0.0, longitude=0.0),
        filtered_wind_sensor=WindSensor(),
        planner="rrtstar",
    )

    updated_obstacles = ompl_path.OMPLPath.init_obstacles(
        local_path_state=updated_local_path_state, state_space_xy=state_space_xy
    )

    assert set(updated_obstacles.keys()) == {LAND_KEY, 1, 3}
    # Check that boat 1 was updated, not replaced
    assert old_boat1 is updated_obstacles[1]
    # Check that boat 2 is evicted
    assert 2 not in updated_obstacles
    # Check that boat 3 is added
    assert isinstance(updated_obstacles[3], ob.Boat)


@pytest.mark.parametrize(
    "x,y,is_valid",
    [(0.5, 0.5, True), (-14, 0.5, False), (-16, 0.5, True)],
)
def test_is_state_valid(x: float, y: float, is_valid: bool, fresh_ompl_path):
    state = base.State(fresh_ompl_path._simple_setup.getStateSpace())
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

    ompl_path.OMPLPath.obstacles = {}
    ompl_path.OMPLPath.obstacles[1] = boat1

    if is_valid:
        assert ompl_path.OMPLPath.is_state_valid(state), "state should be valid"
    else:
        assert not ompl_path.OMPLPath.is_state_valid(state), "state should not be valid"


@pytest.mark.parametrize(
    "position,expected_area,expected_bounds,box_buffer_size",
    [
        (cs.XY(0.0, 0.0), pytest.approx(4, rel=1e-2), (-1, -1, 1, 1), 1.0),
        (cs.XY(100.0, 100.0), pytest.approx(4, rel=1e-2), (99, 99, 101, 101), 1.0),
        (cs.XY(-50.0, -50.0), pytest.approx(4, rel=1e-2), (-51, -51, -49, -49), 1.0),
        (cs.XY(100.0, 100.0), pytest.approx(36, rel=1e-2), (97, 97, 103, 103), 3.0),
        (cs.XY(-50.0, -50.0), pytest.approx(36, rel=1e-2), (-53, -53, -47, -47), 3.0),
    ],
)
def test_create_space(
    position: cs.XY, expected_area, expected_bounds, box_buffer_size: float, fresh_ompl_path
):
    """Test creation of buffered space around positions"""
    # Given an OMPLPath instance

    space = fresh_ompl_path.create_buffer_around_position(position, box_buffer_size)

    assert space.area == expected_area, "Space area should match buffer size"
    assert space.bounds == pytest.approx(
        expected_bounds, abs=box_buffer_size
    ), "Bounds should match expected"
    assert space.contains(Point(position.x, position.y)), "Space should contain center point"


@pytest.mark.parametrize("boat_latlon", [HelperLatLon(latitude=0.0, longitude=0.0)])
def test_get_remaining_cost_full_path(fresh_ompl_path, boat_latlon):
    assert pytest.approx(
        fresh_ompl_path.get_remaining_cost(0, boat_latlon), fresh_ompl_path.get_cost(), 0.01
    )


@pytest.mark.parametrize(
    "boat_latlon", [HelperLatLon(latitude=0.05, longitude=0.04)]  # Halfway through the path
)
def test_get_remaining_cost(fresh_ompl_path, boat_latlon):
    cost = fresh_ompl_path.get_remaining_cost(0, boat_latlon)
    cost_from_next_wp = fresh_ompl_path.get_remaining_cost(1, boat_latlon)
    full_cost = fresh_ompl_path.get_cost()
    assert cost < full_cost, f"Remaining cost {cost} should be less than full cost {full_cost}"
    assert cost > cost_from_next_wp, (
        f"Cost from current waypoint {cost} should be greater than "
        f"cost from next waypoint {cost_from_next_wp}"
    )
