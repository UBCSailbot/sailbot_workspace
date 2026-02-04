import pickle
from typing import Any

import numpy as np
import pytest
from custom_interfaces.msg import (
    HelperAISShip,
    HelperDimension,
    HelperHeading,
    HelperLatLon,
    HelperROT,
    HelperSpeed,
)
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
from local_pathfinding.obstacles import BOAT_BUFFER, Boat, Land, Obstacle


def load_pkl(file_path: str) -> Any:
    with open(file_path, "rb") as f:
        return pickle.load(f)


LAND = MultiPolygon()

try:
    LAND = load_pkl("/workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/land.pkl")
except RuntimeError as e:
    exit(f"could not load the land.pkl file {e}")


# LAND OBSTACLES ----------------------------------------------------------------------------------
@pytest.mark.parametrize(
    "reference_point, sailbot_position, all_land_data, bbox_buffer_amount, land_present",  # noqa
    [
        (
            HelperLatLon(latitude=48.927646856442834, longitude=-125.18555198866946),
            HelperLatLon(latitude=48.842045056421135, longitude=-125.29181185529734),
            LAND,
            0.1,  # degrees
            True,
        ),
        (
            HelperLatLon(latitude=44.112832, longitude=-156.008729),
            HelperLatLon(latitude=44.112832, longitude=-151.260136),
            LAND,
            0.1,  # degrees
            False,
        ),
    ],
)
def test_create_land(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    all_land_data: MultiPolygon,
    bbox_buffer_amount: float,
    land_present: bool,
):

    goal_position = reference_point

    # create the xy state space from the specified positions of sailbot and the goal
    sailbot_box = Point(sailbot_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    goal_box = Point(goal_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    state_space_latlon = box(*MultiPolygon([sailbot_box, goal_box]).bounds)

    land = Land(
        reference=reference_point,
        sailbot_position=sailbot_position,
        all_land_data=all_land_data,
        bbox_buffer_amount=bbox_buffer_amount,
        state_space_latlon=state_space_latlon,
    )

    assert isinstance(land.collision_zone, MultiPolygon)
    if land_present:
        assert len(land.collision_zone.geoms) != 0  # type: ignore
    else:
        assert len(land.collision_zone.geoms) == 0  # type: ignore


def test_create_land_no_state_space():
    with pytest.raises(ValueError):
        Land(
            reference=HelperLatLon(),
            sailbot_position=HelperLatLon(),
            all_land_data=LAND,
            bbox_buffer_amount=0.1,
        )


# Test is_valid
@pytest.mark.parametrize(
    "reference_point, sailbot_position, all_land_data, bbox_buffer_amount, invalid_point, valid_point, mock_land",  # noqa
    [
        (
            HelperLatLon(latitude=48.541341, longitude=-127.424606),
            HelperLatLon(latitude=51.95, longitude=-136.26),
            LAND,
            0.1,  # degrees
            cs.XY(0, 0),
            cs.XY(100, 100),
            MultiPolygon(
                [
                    Polygon(
                        [
                            Point([-127.336762, 49.220467]),
                            Point([-128.401875, 49.047950]),
                            Point([-127.929711, 48.308044]),
                            Point([-126.568123, 48.526792]),
                        ]
                    ),
                ]
            ),
        )
    ],
)
def test_is_valid_land(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    all_land_data: MultiPolygon,
    bbox_buffer_amount: float,
    invalid_point: cs.XY,
    valid_point: cs.XY,
    mock_land: MultiPolygon,
):
    land = Land(
        reference=reference_point,
        sailbot_position=sailbot_position,
        all_land_data=all_land_data,
        bbox_buffer_amount=bbox_buffer_amount,
        land_multi_polygon=mock_land,
    )

    assert land.is_valid(valid_point)
    assert not land.is_valid(invalid_point)


# Test land collision zone is created/updated successfully
@pytest.mark.parametrize(
    "reference_point, sailbot_position, all_land_data, bbox_buffer_amount",
    [
        (
            HelperLatLon(latitude=48.927646856442834, longitude=-125.18555198866946),
            HelperLatLon(latitude=48.842045056421135, longitude=-125.29181185529734),
            LAND,
            0.1,  # degrees
        )
    ],
)
def test_collision_zone_land(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    all_land_data: MultiPolygon,
    bbox_buffer_amount,
):
    goal_position = reference_point

    # create the xy state space from the specified positions of sailbot and the goal
    sailbot_box = Point(sailbot_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    goal_box = Point(goal_position.longitude, sailbot_position.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    state_space_latlon = box(*MultiPolygon([sailbot_box, goal_box]).bounds)

    land = Land(
        reference=reference_point,
        sailbot_position=sailbot_position,
        all_land_data=all_land_data,
        bbox_buffer_amount=bbox_buffer_amount,
        state_space_latlon=state_space_latlon,
    )
    land.update_collision_zone(state_space_latlon=state_space_latlon)

    assert isinstance(land.collision_zone, MultiPolygon)
    assert len(land.collision_zone.geoms) != 0


# Test updating Sailbot data
@pytest.mark.parametrize(
    "reference_point, sailbot_position_1, sailbot_position_2, all_land_data, bbox_buffer_amount",  # noqa
    [
        (
            HelperLatLon(latitude=52.26, longitude=-136.91),
            HelperLatLon(latitude=51.0, longitude=-136.0),
            HelperLatLon(latitude=52.0, longitude=-137.0),
            LAND,
            0.1,  # degrees
        )
    ],
)
def test_update_sailbot_data_land(
    reference_point: HelperLatLon,
    sailbot_position_1: HelperLatLon,
    sailbot_position_2: HelperLatLon,
    all_land_data: MultiPolygon,
    bbox_buffer_amount,
):

    # create the xy state space from the specified positions of sailbot and the goal
    goal_position = reference_point
    sailbot_box = Point(sailbot_position_1.longitude, sailbot_position_1.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    goal_box = Point(goal_position.longitude, sailbot_position_1.latitude).buffer(
        0.1, cap_style=3, join_style=2
    )

    state_space_latlon = box(*MultiPolygon([sailbot_box, goal_box]).bounds)

    land = Land(
        reference=reference_point,
        sailbot_position=sailbot_position_1,
        all_land_data=all_land_data,
        bbox_buffer_amount=bbox_buffer_amount,
        state_space_latlon=state_space_latlon,
    )

    land.update_sailbot_data(sailbot_position_2)
    assert land.sailbot_position == pytest.approx(
        cs.latlon_to_xy(reference_point, sailbot_position_2)
    )


# Test update reference point
@pytest.mark.parametrize(
    "reference_point_1, reference_point_2, sailbot_position, all_land_data, bbox_buffer_amount",  # noqa
    [
        (
            HelperLatLon(latitude=49.155485, longitude=-126.987704),
            HelperLatLon(latitude=49.1, longitude=-126.1),
            HelperLatLon(latitude=48.838328, longitude=-126.380390),
            LAND,
            0.1,  # degrees
        ),
    ],
)
def test_update_reference_point_land(
    reference_point_1: HelperLatLon,
    reference_point_2: HelperLatLon,
    sailbot_position: HelperLatLon,
    all_land_data: MultiPolygon,
    bbox_buffer_amount,
):
    # Force the state space to be entire world so that the statespace does not change size
    # when the reference point is updated
    # otherwise the land obstacle would have a different shape and we couldn't check if it
    # was translated properly
    state_space_latlon = box(-180, -90, 180, 90)
    land = Land(
        reference=reference_point_1,
        sailbot_position=sailbot_position,
        all_land_data=all_land_data,
        bbox_buffer_amount=bbox_buffer_amount,
        state_space_latlon=state_space_latlon,
    )

    assert land.reference == reference_point_1
    assert land.sailbot_position == pytest.approx(
        cs.latlon_to_xy(reference_point_1, sailbot_position)
    )
    centroid1 = land.collision_zone.centroid  # type: ignore

    land.update_reference_point(reference=reference_point_2, state_space_latlon=state_space_latlon)
    assert land.reference == reference_point_2
    assert land.sailbot_position_latlon == sailbot_position
    assert land.sailbot_position == pytest.approx(
        cs.latlon_to_xy(reference_point_2, sailbot_position)
    )

    centroid2 = land.collision_zone.centroid  # type: ignore

    # Calculate the expected displacement based on the old and new reference point
    x_displacement, y_displacement = cs.latlon_to_xy(reference_point_2, reference_point_1)
    displacement = np.sqrt(x_displacement**2 + y_displacement**2)
    # calculate how far the collision zone was actually translated on reference point update
    translation = centroid1.distance(centroid2)
    assert translation == pytest.approx(displacement, rel=0.1), "incorrect translation"


# BOAT OBSTACLES ----------------------------------------------------------------------------------


# Test calculate projected distance
# Boat and Sailbot in same location
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.957, longitude=-136.262),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.957, longitude=-136.262),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        )
    ],
)
def test_calculate_projected_distance_same_loc(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    cog_rad = np.radians(ais_ship.cog.heading)

    assert boat1._calculate_projected_distance(cog_rad) == pytest.approx(
        0.0
    ), "incorrect projected distance"


# Test calculate projected distance
# Boat and Sailbot in different locations
@pytest.mark.parametrize(
    "reference_point_latlon,sailbot_position_latlon,ais_ship,sailbot_speed_kmph",
    [
        (
            HelperLatLon(latitude=49.283075, longitude=-123.216004),
            HelperLatLon(latitude=49.283439, longitude=-123.209825),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=49.284671, longitude=-123.203216),
                cog=HelperHeading(heading=-60.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        )
    ],
)
def test_calculate_projected_distance_diff_loc(
    reference_point_latlon: HelperLatLon,
    sailbot_position_latlon: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed_kmph: float,
):
    target_ship = Boat(
        reference_point_latlon, sailbot_position_latlon, sailbot_speed_kmph, ais_ship
    )
    cog_rad = np.radians(ais_ship.cog.heading)
    target_ship_dist_km = target_ship._calculate_projected_distance(cog_rad)
    travel_time_hr = target_ship_dist_km / ais_ship.sog.speed
    target_ship_speed_x_kmph = np.sin(np.radians(ais_ship.cog.heading)) * ais_ship.sog.speed
    target_ship_speed_y_kmph = np.cos(np.radians(ais_ship.cog.heading)) * ais_ship.sog.speed
    target_ship_initial_point_xy = cs.latlon_to_xy(
        reference=reference_point_latlon, latlon=ais_ship.lat_lon
    )
    dx = target_ship_speed_x_kmph * travel_time_hr
    dy = target_ship_speed_y_kmph * travel_time_hr

    target_ship_final_point_xy = Point(
        target_ship_initial_point_xy.x + dx, target_ship_initial_point_xy.y + dy
    )

    sailbot_pos_xy = Point(
        *cs.latlon_to_xy(reference=reference_point_latlon, latlon=sailbot_position_latlon)
    )
    sailbot_dist_km = sailbot_speed_kmph * travel_time_hr

    assert sailbot_pos_xy.distance(target_ship_final_point_xy) == pytest.approx(
        sailbot_dist_km, abs=0.001
    )


# Test collision zone for straight line case
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        )
    ],
)
def test_collision_zone_straight_line_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    boat1.update_collision_zone()

    assert isinstance(boat1.collision_zone, Polygon)
    assert boat1.collision_zone is not None
    assert boat1.collision_zone.exterior.coords is not None


# Test collision zone for no collision case
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.957, longitude=-136.262),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.957, longitude=-136.262),  # Same as sailbot
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        )
    ],
)
def test_collision_zone_no_collision_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    boat1.update_collision_zone()

    assert isinstance(boat1.collision_zone, Polygon)
    assert boat1.collision_zone is not None
    # Circular zone should have many vertices
    assert len(boat1.collision_zone.exterior.coords) > 10


# Test collision zone for turning motion case
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=20),
            ),
            15.0,
        ),
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=-20),
            ),
            15.0,
        ),
    ],
)
def test_collision_zone_turning_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    boat1.update_collision_zone()

    assert isinstance(boat1.collision_zone, Polygon)
    assert boat1.collision_zone is not None
    assert boat1.collision_zone.exterior.coords is not None


# COLLISION ZONE LOGIC


# Test collision zone positioning for straight-line motion
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.0, longitude=-136.0),
            HelperLatLon(latitude=52.01, longitude=-136.0),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=52.0, longitude=-136.0),
                cog=HelperHeading(heading=0.0),  # north
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        ),
        (
            HelperLatLon(latitude=52.0, longitude=-136.0),
            HelperLatLon(latitude=52.0, longitude=-135.99),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=52.0, longitude=-136.0),
                cog=HelperHeading(heading=90.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        ),
    ],
)
def test_straight_line_collision_zone_geometry(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)

    if boat1._raw_collision_zone is not None:
        unbuffered = boat1._raw_collision_zone

        stern_x = -cs.meters_to_km(boat1.ais_ship.width.dimension) / 2
        stern_y = -cs.meters_to_km(boat1.ais_ship.length.dimension) / 2

        dx, dy = cs.latlon_to_xy(reference_point, ais_ship.lat_lon)
        angle = np.radians(-ais_ship.cog.heading)
        cos_t, sin_t = np.cos(angle), np.sin(angle)

        stern_world = np.array(
            [stern_x * cos_t - stern_y * sin_t + dx, stern_x * sin_t + stern_y * cos_t + dy]
        )

        coords = np.array(unbuffered.exterior.coords)
        dists = np.linalg.norm(coords - stern_world, axis=1)
        closest = coords[np.argmin(dists)]

        assert np.allclose(closest, stern_world, atol=0.001)


# Test collision zone positioning for turning motion
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.0, longitude=-136.0),
            HelperLatLon(latitude=52.002, longitude=-135.998),  # ahead + right
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=52.0, longitude=-136.0),
                cog=HelperHeading(heading=0.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=30),  # turning right
            ),
            15.0,
        ),
    ],
)
def test_turning_collision_zone_geometry(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)

    if boat._raw_collision_zone is not None:

        unbuffered = boat._raw_collision_zone
        coords = np.array(unbuffered.exterior.coords[:-1])
        assert coords.shape == (3, 2)

        bow_y = cs.meters_to_km(ais_ship.length.dimension) / 2
        cog_rad = np.radians(ais_ship.cog.heading)

        projected_distance = boat._calculate_projected_distance(cog_rad)

        rot = ais_ship.rot.rot
        rot_rps = cs.rot_to_rad_per_sec(rot)
        dt = 5.0
        delta_heading = rot_rps * dt
        future_cog_rad = cog_rad + delta_heading

        # Compute future boat position
        speed_kmps = ais_ship.sog.speed / 3600.0
        R = speed_kmps / abs(rot_rps)
        x, y = cs.latlon_to_xy(reference_point, ais_ship.lat_lon)

        cx = x - R * np.sin(cog_rad) * np.sign(rot_rps)
        cy = y + R * np.cos(cog_rad) * np.sign(rot_rps)

        future_x = cx + R * np.sin(cog_rad + delta_heading)
        future_y = cy + R * np.cos(cog_rad + delta_heading)

        future_projected_distance = boat._calculate_projected_distance(
            future_cog_rad, position_override=(future_x, future_y)
        )

        A = np.array([0.0, bow_y])
        B = np.array([0.0, bow_y + projected_distance])
        C = np.array(
            [
                future_projected_distance * np.sin(delta_heading),
                bow_y + future_projected_distance * np.cos(delta_heading),
            ]
        )

        expected_local = np.vstack([A, B, C])

        dx, dy = cs.latlon_to_xy(reference_point, ais_ship.lat_lon)
        angle_rad = np.radians(-ais_ship.cog.heading)
        sin_t = np.sin(angle_rad)
        cos_t = np.cos(angle_rad)

        T = np.array(
            [
                [cos_t, -sin_t],
                [sin_t, cos_t],
            ]
        )

        rotated = np.matmul(expected_local, T.T)
        translation = np.array([dx, dy])
        expected_world = rotated + translation
        coords_sorted = coords[np.lexsort((coords[:, 0], coords[:, 1]))]
        expected_sorted = expected_world[np.lexsort((expected_world[:, 0], expected_world[:, 1]))]

        assert np.allclose(coords_sorted, expected_sorted, atol=1e-6)


# Test create collision zone raises error when id of passed ais_ship does not match self's id
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship_1,ais_ship_2,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            HelperAISShip(
                id=2,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        )
    ],
)
def test_create_collision_zone_id_mismatch_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship_1: HelperAISShip,
    ais_ship_2: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship_1)

    with pytest.raises(ValueError):
        boat1.update_collision_zone(ais_ship=ais_ship_2)


# Test is_valid
@pytest.mark.parametrize(
    "reference_point,sailbot_position,ais_ship,sailbot_speed,invalid_point,valid_point",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=0.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
            cs.latlon_to_xy(
                HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
                HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
            ),
            cs.latlon_to_xy(
                HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
                HelperLatLon(latitude=49.30499213908291, longitude=-123.31330140816111),
            ),
        )
    ],
)
def test_is_valid_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
    invalid_point: cs.XY,
    valid_point: cs.XY,
):
    boat1 = Boat(reference_point, sailbot_position, sailbot_speed, ais_ship)
    assert not boat1.is_valid(invalid_point)
    assert boat1.is_valid(valid_point)


# Test is_valid raises error when collision zone has not been set
@pytest.mark.parametrize(
    "reference_point,sailbot_position,invalid_point,valid_point",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            cs.latlon_to_xy(
                HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
                HelperLatLon(latitude=52.174842845359755, longitude=-137.10372451905042),
            ),
            cs.latlon_to_xy(
                HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
                HelperLatLon(latitude=49.30499213908291, longitude=-123.31330140816111),
            ),
        )
    ],
)
def test_is_valid_no_collision_zone_boat(
    reference_point: HelperLatLon,
    sailbot_position: HelperLatLon,
    invalid_point: cs.XY,
    valid_point: cs.XY,
):
    obstacle = Obstacle(reference_point, sailbot_position)  # type: ignore
    with pytest.raises(RuntimeError):
        obstacle.is_valid(invalid_point)
    with pytest.raises(RuntimeError):
        obstacle.is_valid(valid_point)


# Test updating Sailbot data
@pytest.mark.parametrize(
    "ref_point,sailbot_position_1,sailbot_speed_1,sailbot_position_2,sailbot_speed_2,ais_ship",
    [
        (
            HelperLatLon(latitude=52.268119490007756, longitude=-136.9133983613776),
            HelperLatLon(latitude=51.9, longitude=-136.2),
            15.0,
            HelperLatLon(latitude=52.9, longitude=-137.2),
            20.0,
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
        )
    ],
)
def test_update_sailbot_data_boat(
    ref_point: HelperLatLon,
    sailbot_position_1: HelperLatLon,
    sailbot_speed_1: float,
    sailbot_position_2: HelperLatLon,
    sailbot_speed_2: float,
    ais_ship: HelperAISShip,
):
    boat1 = Boat(ref_point, sailbot_position_1, sailbot_speed_1, ais_ship)
    boat1.update_sailbot_data(sailbot_position_2, sailbot_speed_2)

    assert boat1.sailbot_position == pytest.approx(cs.latlon_to_xy(ref_point, sailbot_position_2))
    assert boat1.sailbot_speed == pytest.approx(sailbot_speed_2)


# Test update reference point
@pytest.mark.parametrize(
    "reference_point_1,reference_point_2,sailbot_position,ais_ship,sailbot_speed",
    [
        (
            HelperLatLon(latitude=52.2, longitude=-136.9),
            HelperLatLon(latitude=51.0, longitude=-136.0),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        ),
        (
            HelperLatLon(latitude=50.06442134644842, longitude=-130.7725487868677),
            HelperLatLon(latitude=49.88670956993386, longitude=-130.37061359404225),
            HelperLatLon(latitude=51.95785651405779, longitude=-136.26282894969611),
            HelperAISShip(
                id=1,
                lat_lon=HelperLatLon(latitude=51.97917631092298, longitude=-137.1106454702385),
                cog=HelperHeading(heading=30.0),
                sog=HelperSpeed(speed=20.0),
                width=HelperDimension(dimension=20.0),
                length=HelperDimension(dimension=100.0),
                rot=HelperROT(rot=0),
            ),
            15.0,
        ),
    ],
)
def test_update_reference_point_boat(
    reference_point_1: HelperLatLon,
    reference_point_2: HelperLatLon,
    sailbot_position: HelperLatLon,
    ais_ship: HelperAISShip,
    sailbot_speed: float,
):
    boat1 = Boat(reference_point_1, sailbot_position, sailbot_speed, ais_ship)
    if isinstance(boat1.collision_zone, Polygon):
        point1 = Point(
            boat1.collision_zone.exterior.coords.xy[0][0],
            boat1.collision_zone.exterior.coords.xy[1][0],
        )

    assert boat1.reference == reference_point_1
    assert boat1.sailbot_position == pytest.approx(
        cs.latlon_to_xy(reference_point_1, sailbot_position)
    )
    # Change the reference point
    boat1.update_reference_point(reference_point_2)
    if isinstance(boat1.collision_zone, Polygon):
        point2 = Point(
            boat1.collision_zone.exterior.coords.xy[0][0],
            boat1.collision_zone.exterior.coords.xy[1][0],
        )

    assert boat1.reference == reference_point_2
    assert boat1.sailbot_position_latlon == sailbot_position
    assert boat1.sailbot_position == pytest.approx(
        cs.latlon_to_xy(reference_point_2, sailbot_position)
    )

    # Calculate the expected displacement based on the old and new reference point
    x_displacement, y_displacement = cs.latlon_to_xy(reference_point_2, reference_point_1)
    displacement = np.sqrt(x_displacement**2 + y_displacement**2)
    # calculate how far the collision zone was actually translated on reference point update
    translation = point1.distance(point2)

    # There is some error in the latlon_to_xy conversion but the results are close
    assert translation == pytest.approx(displacement, rel=0.1), "incorrect translation"
