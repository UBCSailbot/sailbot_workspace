import math
import random
import types
from types import SimpleNamespace

import pytest
from ompl import base
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Point, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
import local_pathfinding.ompl_path as ompl_path
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
from local_pathfinding.local_path import LocalPathState, WindTracker
from local_pathfinding.wind_coord_systems import Wind

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
                    HelperLatLon(latitude=0.1, longitude=0.1),
                    HelperLatLon(latitude=0.15, longitude=0.15),
                    HelperLatLon(latitude=0.12, longitude=0.11),
                    HelperLatLon(latitude=0.10, longitude=0.09),
                    HelperLatLon(latitude=0.08, longitude=0.07),
                    HelperLatLon(latitude=0.06, longitude=0.05),
                    HelperLatLon(latitude=0.04, longitude=0.03),
                    HelperLatLon(latitude=0.02, longitude=0.02),
                ]
            ),
            target_global_waypoint=HelperLatLon(latitude=0.02, longitude=0.02),
            filtered_wind_sensor=WindSensor(),
            planner="rrtstar",
            wind_tracker=WindTracker(),
        ),
        should_simplify_path=False,
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
        wind_tracker=WindTracker(),
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
        wind_tracker=WindTracker(),
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
    "distance_from_reference_km,expected_pkl_path",
    [
        (0.0, ompl_path.ON_WATER_LAND_PKL_FILE_PATH),  # at the reference -> on-water
        (10.0, ompl_path.ON_WATER_LAND_PKL_FILE_PATH),  # within threshold -> on-water
        (
            ompl_path.DISTANCE_FROM_ON_WATER_LANDMARK - 0.1,
            ompl_path.ON_WATER_LAND_PKL_FILE_PATH,
        ),  # just inside threshold -> on-water
        (
            ompl_path.DISTANCE_FROM_ON_WATER_LANDMARK + 0.1,
            ompl_path.OFFSHORE_LAND_PKL_FILE_PATH,
        ),  # just past threshold -> offshore
        (100.0, ompl_path.OFFSHORE_LAND_PKL_FILE_PATH),  # far offshore -> offshore
    ],
)
def test_load_appropriate_land_obstacle(
    distance_from_reference_km, expected_pkl_path, monkeypatch
):
    """The dataset is selected by distance from the on-water reference, not by file contents."""
    # place a position the requested distance away from the on-water reference
    ref_lat, ref_lon = cs.ON_WATER_REFERENCE
    lon, lat, _ = cs.GEODESIC.fwd(
        lons=ref_lon, lats=ref_lat, az=0.0, dist=distance_from_reference_km * 1000
    )
    local_path_state = types.SimpleNamespace(position=HelperLatLon(latitude=lat, longitude=lon))

    # avoid depending on the real .pkl contents: record the path that would be loaded
    loaded_paths = []

    def fake_load_pkl(file_path):
        loaded_paths.append(file_path)
        return MultiPolygon()

    monkeypatch.setattr(ompl_path, "load_pkl", fake_load_pkl)

    try:
        ompl_path.OMPLPath.load_appropriate_land_obstacle(local_path_state)

        assert loaded_paths == [expected_pkl_path], "loaded the wrong land dataset"
        assert ompl_path.OMPLPath.all_land_data is not None
    finally:
        # reset shared static state so other tests reload land data as needed
        ompl_path.OMPLPath.all_land_data = None


def test_load_appropriate_land_obstacle_missing_file_warns(monkeypatch):
    """A missing land .pkl file logs a warning and falls back to an empty MultiPolygon."""
    local_path_state = types.SimpleNamespace(position=HelperLatLon(latitude=0.0, longitude=0.0))

    def raise_not_found(file_path):
        raise FileNotFoundError(file_path)

    monkeypatch.setattr(ompl_path, "load_pkl", raise_not_found)

    ompl_path.OMPLPath.load_appropriate_land_obstacle(local_path_state)
    assert ompl_path.OMPLPath.all_land_data == MultiPolygon()


@pytest.mark.parametrize(
    "x,y,is_valid",
    [(0.5, 0.5, True), (-13.5, 0.5, False), (-16, 0.5, True)],
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
        assert fresh_ompl_path.is_state_valid(state), "state should be valid"
    else:
        assert not fresh_ompl_path.is_state_valid(state), "state should not be valid"


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
    remaining_cost = fresh_ompl_path.get_remaining_cost(1, boat_latlon)
    assert remaining_cost == pytest.approx(fresh_ompl_path.get_cost(), abs=0.01)


@pytest.mark.parametrize(
    "target_wp_index",
    [
        1,
        2,
        3,
        4,
    ],
)
def test_get_remaining_cost_partial(fresh_ompl_path, target_wp_index):
    waypoints = fresh_ompl_path.get_path().waypoints
    current_wp_latlon = waypoints[target_wp_index - 1]
    next_wp_latlon = waypoints[target_wp_index]

    def mid_point(start_latlon: HelperLatLon, end_latlon: HelperLatLon):
        """Return a random point between waypoints."""
        if end_latlon is None:
            return start_latlon
        end_points_inclusive_factor = 1e-4
        return HelperLatLon(
            latitude=(
                random.uniform(
                    start_latlon.latitude + end_points_inclusive_factor,
                    end_latlon.latitude - end_points_inclusive_factor,
                )
            ),
            longitude=(
                random.uniform(
                    start_latlon.longitude + end_points_inclusive_factor,
                    end_latlon.longitude - end_points_inclusive_factor,
                )
            ),  # noqa
        )

    boat_latlon = mid_point(current_wp_latlon, next_wp_latlon)
    cost = fresh_ompl_path.get_remaining_cost(target_wp_index, boat_latlon)

    # cannot calculate cost_from_next_wp as index out of bound for final waypoint
    if target_wp_index == len(waypoints) - 1:
        with pytest.raises(Exception):
            fresh_ompl_path.get_remaining_cost(target_wp_index + 1, next_wp_latlon)
        return

    cost_from_next_wp = fresh_ompl_path.get_remaining_cost(target_wp_index + 1, next_wp_latlon)

    full_cost = fresh_ompl_path.get_cost()
    assert (
        cost <= full_cost
    ), f"Remaining cost {cost} should be less than or equal to full cost {full_cost}"
    assert cost > cost_from_next_wp, (
        f"Cost from waypoint {target_wp_index} ({cost}) should be greater than "
        f"cost from waypoint {target_wp_index + 1} ({cost_from_next_wp})"
    )


@pytest.mark.parametrize(
    "target_wp_index,",
    [
        1,
        2,
        3,
        4,
    ],
)
def test_get_remaining_cost_no_partial(fresh_ompl_path, target_wp_index):
    waypoints = fresh_ompl_path.get_path().waypoints
    boat_latlon = waypoints[target_wp_index - 1]
    cost = fresh_ompl_path.get_remaining_cost(target_wp_index, boat_latlon)
    next_wp_latlon = waypoints[target_wp_index]

    if target_wp_index == len(waypoints) - 1:
        with pytest.raises(Exception):
            fresh_ompl_path.get_remaining_cost(target_wp_index + 1, next_wp_latlon)
        return

    cost_from_next_wp = fresh_ompl_path.get_remaining_cost(target_wp_index + 1, next_wp_latlon)

    full_cost = fresh_ompl_path.get_cost()
    assert (
        cost <= full_cost
    ), f"Remaining cost {cost} should be less than or equal to full cost {full_cost}"
    assert cost > cost_from_next_wp, (
        f"Cost from waypoint {target_wp_index} ({cost}) should be greater than "
        f"cost from waypoint {target_wp_index + 1} ({cost_from_next_wp})"
    )


def test_get_remaining_cost_projection_logic(fresh_ompl_path):
    """
    Test that get_remaining_cost uses projection logic correctly.

    This test verifies that even when the boat is farther from the segment end
    than the segment start is (e.g., boat is off to the side), the projection
    logic correctly determines progress along the segment.
    """
    waypoints = fresh_ompl_path.get_path().waypoints

    # Use segment from waypoint 1 to waypoint 2 (target index = 2)
    target_wp_index = 2
    start_latlon = waypoints[target_wp_index - 1]
    end_latlon = waypoints[target_wp_index]

    # Convert to XY coordinates for easier geometric calculations
    reference = fresh_ompl_path.state.reference_latlon
    start_xy = cs.latlon_to_xy(reference, start_latlon)
    end_xy = cs.latlon_to_xy(reference, end_latlon)

    # Calculate segment vector
    seg_dx = end_xy.x - start_xy.x
    seg_dy = end_xy.y - start_xy.y
    seg_length = math.sqrt(seg_dx**2 + seg_dy**2)

    # Create a boat position at 50% along the segment, but offset perpendicular
    # This makes the boat farther from the end than the segment length
    progress = 0.5
    boat_along_x = start_xy.x + progress * seg_dx
    boat_along_y = start_xy.y + progress * seg_dy

    # Add perpendicular offset (perpendicular to segment direction)
    # Perpendicular vector: (-dy, dx) normalized
    perp_scale = seg_length  # Offset by the segment length
    perp_x = -seg_dy / seg_length * perp_scale
    perp_y = seg_dx / seg_length * perp_scale

    boat_xy = cs.XY(boat_along_x + perp_x, boat_along_y + perp_y)

    # Convert back to lat/lon
    boat_latlon = cs.xy_to_latlon(reference, boat_xy)

    # Verify that boat is farther from end than segment length
    boat_to_end_dist = math.sqrt((end_xy.x - boat_xy.x) ** 2 + (end_xy.y - boat_xy.y) ** 2)
    assert boat_to_end_dist > seg_length, (
        "Test setup: boat should be farther from end than segment length "
        f"(boat_to_end={boat_to_end_dist:.3f}, seg_length={seg_length:.3f})"
    )

    # Get remaining cost from this position
    remaining_cost = fresh_ompl_path.get_remaining_cost(target_wp_index, boat_latlon)

    # Get cost from the start and end of the segment
    cost_from_start = fresh_ompl_path.get_remaining_cost(target_wp_index, start_latlon)
    cost_from_end = fresh_ompl_path.get_remaining_cost(target_wp_index + 1, end_latlon)

    # The remaining cost should be between the cost from start and cost from end
    # since we're 50% along the segment
    assert cost_from_end < remaining_cost < cost_from_start, (
        f"Remaining cost from projected position ({remaining_cost:.3f}) should be between "
        f"cost from start ({cost_from_start:.3f}) and cost from end ({cost_from_end:.3f})"
    )


# ---------------------------------------------------------------------------
# Goal-yaw tests for _compute_goal_heading_rad and _offset_if_in_irons
# ---------------------------------------------------------------------------


def _yaw_test_self(waypoints, target, tw_dir_deg, tw_speed_kmph=10.0):
    """Minimal stand-in for an OMPLPath instance, exposing only the attributes the
    goal-yaw helpers read. Avoids running the OMPL solver in every test.
    """
    wind = Wind(tw_speed_kmph, tw_dir_deg)
    state = SimpleNamespace(
        global_path=Path(waypoints=waypoints),
        reference_latlon=target,
        path_generated_wind=wind,
        current_tw=wind,
    )
    fake = SimpleNamespace(state=state, _logger=RcutilsLogger())
    # Bind the helpers so `_compute_goal_heading_rad` can call `self._offset_if_in_irons`
    fake._offset_if_in_irons = types.MethodType(ompl_path.OMPLPath._offset_if_in_irons, fake)
    return fake


# --- _compute_goal_heading_rad: bearing toward next-to-next waypoint ---------


def test_compute_goal_heading_rad_next_next_east():
    """Yaw should point east when the next-to-next waypoint is east of the next one."""
    target = HelperLatLon(latitude=0.0, longitude=0.0)
    next_next_east = HelperLatLon(latitude=0.0, longitude=1.0)
    # waypoints are in reverse order; target is not the final destination
    waypoints = [next_next_east, target]
    # wind from north → east heading is well outside the no-go cones
    fake = _yaw_test_self(waypoints, target, tw_dir_deg=0.0)

    yaw = ompl_path.OMPLPath._compute_goal_heading_rad(fake)

    # East in OMPL cartesian is 0 rad
    assert yaw == pytest.approx(0.0, abs=1e-6)


def test_compute_goal_heading_rad_next_next_north():
    """Yaw should point north when the next-to-next waypoint is north of the next one."""
    target = HelperLatLon(latitude=0.0, longitude=0.0)
    next_next_north = HelperLatLon(latitude=1.0, longitude=0.0)
    waypoints = [next_next_north, target]
    # wind from east → north heading is well outside the no-go cones
    fake = _yaw_test_self(waypoints, target, tw_dir_deg=90.0)

    yaw = ompl_path.OMPLPath._compute_goal_heading_rad(fake)

    # North in OMPL cartesian is +pi/2 rad
    assert yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_compute_goal_heading_rad_next_next_south():
    """Yaw should point south when the next-to-next waypoint is south of the next one."""
    target = HelperLatLon(latitude=0.0, longitude=0.0)
    next_next_south = HelperLatLon(latitude=-1.0, longitude=0.0)
    waypoints = [next_next_south, target]
    # wind from west → south heading is well outside the no-go cones
    fake = _yaw_test_self(waypoints, target, tw_dir_deg=-90.0)

    yaw = ompl_path.OMPLPath._compute_goal_heading_rad(fake)

    # South in OMPL cartesian is -pi/2 rad
    assert yaw == pytest.approx(-math.pi / 2, abs=1e-6)


def test_compute_goal_heading_rad_final_destination_returns_default():
    """When the next global waypoint IS the final destination (waypoints[0]),
    the yaw defaults to 0.0 (east) as long as it does not land in irons."""
    target = HelperLatLon(latitude=0.0, longitude=0.0)
    other_wp = HelperLatLon(latitude=1.0, longitude=0.0)
    # target sits at index 0 → it is the final destination
    waypoints = [target, other_wp]
    # wind from north → east heading (the default) is safe
    fake = _yaw_test_self(waypoints, target, tw_dir_deg=0.0)

    yaw = ompl_path.OMPLPath._compute_goal_heading_rad(fake)

    assert yaw == pytest.approx(0.0, abs=1e-6)


# --- _offset_if_in_irons: wind no-go zone offsetting -------------------------


def test_offset_if_in_irons_yaw_outside_no_go_returns_unchanged():
    """A yaw outside the wind no-go zone should pass through untouched."""
    fake = _yaw_test_self(
        [HelperLatLon(latitude=1.0, longitude=0.0), HelperLatLon(latitude=0.0, longitude=0.0)],
        HelperLatLon(latitude=0.0, longitude=0.0),
        tw_dir_deg=0.0,  # wind from north
    )
    # East yaw (OMPL 0) makes 90° with wind from north → beam reach, safe
    yaw_in = 0.0

    yaw_out = ompl_path.OMPLPath._offset_if_in_irons(fake, yaw_in)

    assert yaw_out == pytest.approx(yaw_in, abs=1e-6)


def test_offset_if_in_irons_dead_upwind_snaps_to_no_go_edge():
    """A yaw aimed dead upwind should snap to one of the NO_GO_ZONE edges (45° off wind).
    Ambiguous-tie input (twa == 0) breaks toward the +NO_GO_ZONE edge by convention.
    """
    fake = _yaw_test_self(
        [HelperLatLon(latitude=1.0, longitude=0.0), HelperLatLon(latitude=0.0, longitude=0.0)],
        HelperLatLon(latitude=0.0, longitude=0.0),
        tw_dir_deg=0.0,  # wind from north
    )
    # OMPL yaw +pi/2 = boat heading true bearing 0° (north) = dead upwind
    yaw_in = math.pi / 2

    yaw_out = ompl_path.OMPLPath._offset_if_in_irons(fake, yaw_in)

    # Tie-breaker: twa == 0 → new_twa = +NO_GO_ZONE → boat heading = -pi/4 (NW)
    # NW in OMPL cartesian = +3pi/4
    assert yaw_out == pytest.approx(3 * math.pi / 4, abs=1e-6)


def test_offset_if_in_irons_slightly_east_of_upwind_snaps_to_ne_edge():
    """A yaw slightly to the east of dead upwind should snap to the NE edge,
    preserving the easterly bias of the original intent (Option B)."""
    fake = _yaw_test_self(
        [HelperLatLon(latitude=1.0, longitude=0.0), HelperLatLon(latitude=0.0, longitude=0.0)],
        HelperLatLon(latitude=0.0, longitude=0.0),
        tw_dir_deg=0.0,  # wind from north
    )
    # OMPL yaw 85° = true bearing 5° = slightly east of north
    yaw_in = math.radians(85)

    yaw_out = ompl_path.OMPLPath._offset_if_in_irons(fake, yaw_in)

    # twa = -5° (negative) → new_twa = -NO_GO_ZONE → boat heading = +pi/4 (NE)
    # NE in OMPL cartesian = +pi/4
    assert yaw_out == pytest.approx(math.pi / 4, abs=1e-6)


def test_offset_if_in_irons_dead_downwind_snaps_to_no_go_edge():
    """A yaw aimed dead downwind should snap to the broad-reach edge of the downwind cone."""
    fake = _yaw_test_self(
        [HelperLatLon(latitude=1.0, longitude=0.0), HelperLatLon(latitude=0.0, longitude=0.0)],
        HelperLatLon(latitude=0.0, longitude=0.0),
        tw_dir_deg=0.0,  # wind from north
    )
    # OMPL yaw -pi/2 = boat heading true bearing 180° (south) = dead downwind
    yaw_in = -math.pi / 2

    yaw_out = ompl_path.OMPLPath._offset_if_in_irons(fake, yaw_in)

    # twa = pi → new_twa = +(pi - NO_GO_ZONE) → boat heading = -3pi/4 (SW)
    # SW in OMPL cartesian = -3pi/4
    assert yaw_out == pytest.approx(-3 * math.pi / 4, abs=1e-6)


# --- integration: bearing computation that lands in irons --------------------


def test_compute_goal_heading_rad_irons_from_north_snaps():
    """When the desired yaw (toward next-to-next, north) is dead upwind, the result
    should be the snapped no-go edge — verifies the two helpers compose correctly."""
    target = HelperLatLon(latitude=0.0, longitude=0.0)
    next_next_north = HelperLatLon(latitude=1.0, longitude=0.0)
    waypoints = [next_next_north, target]
    fake = _yaw_test_self(waypoints, target, tw_dir_deg=0.0)  # wind from north

    yaw = ompl_path.OMPLPath._compute_goal_heading_rad(fake)

    # Same tie-broken NW edge as the dead-upwind unit test above
    assert yaw == pytest.approx(3 * math.pi / 4, abs=1e-6)


# --- wiring: the computed yaw actually reaches the OMPL goal state -----------


def test_init_simple_setup_applies_goal_yaw(fresh_ompl_path):
    """The yaw produced by _compute_goal_heading_rad must end up on the OMPL goal
    state. Guards against the setYaw call being accidentally removed."""
    expected_yaw = fresh_ompl_path._compute_goal_heading_rad()
    goal_state = fresh_ompl_path._simple_setup.getGoal().getState()
    actual_yaw = goal_state.getYaw()
    assert actual_yaw == pytest.approx(expected_yaw, abs=1e-6)
