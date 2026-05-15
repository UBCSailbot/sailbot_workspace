from datetime import datetime, timedelta
from unittest import mock

import pytest
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.local_path as lp
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from local_pathfinding.obstacles import Obstacle
from local_pathfinding.wind_coord_systems import Wind

REF = HelperLatLon(latitude=10.0, longitude=10.0)

PATH = lp.LocalPath(parent_logger=RcutilsLogger())


@pytest.fixture
def basic_local_path_state():
    gps = mock.Mock()
    gps.lat_lon = HelperLatLon(latitude=0.0, longitude=0.0)
    gps.speed = mock.Mock(speed=0.0)
    gps.heading = mock.Mock(heading=0.0)

    ais_ships = mock.Mock()
    ais_ships.ships = []

    global_path = Path(
        waypoints=[
            HelperLatLon(latitude=0.0, longitude=0.0),
            HelperLatLon(latitude=1.0, longitude=1.0),
        ]
    )

    filtered_wind_sensor = mock.Mock()
    filtered_wind_sensor.speed = mock.Mock(speed=5.0)
    filtered_wind_sensor.direction = 90

    return lp.LocalPathState(
        gps=gps,
        ais_ships=ais_ships,
        global_path=global_path,
        target_global_waypoint=global_path.waypoints[-1],
        filtered_wind_sensor=filtered_wind_sensor,
        planner="rrtstar",
    )


def create_test_local_path_for_in_collision_zone(
    target_lp_wp_index, reference_latlon, path, obstacles
):
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    local_path._target_lp_wp_index = target_lp_wp_index
    local_path.path = path
    local_path.state = mock.Mock(lp.LocalPathState)
    local_path.state.reference_latlon = reference_latlon
    local_path.state.obstacles = obstacles
    return local_path


def create_update_if_needed_inputs():
    gps = mock.Mock()
    gps.lat_lon = HelperLatLon(latitude=0.0, longitude=-0.1)
    gps.speed = mock.Mock(speed=0.0)
    gps.heading = mock.Mock(heading=0.0)

    ais_ships = mock.Mock()
    ais_ships.ships = []

    global_path = Path(
        waypoints=[
            HelperLatLon(latitude=1.0, longitude=1.0),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )

    filtered_wind_sensor = mock.Mock()
    filtered_wind_sensor.speed = mock.Mock(speed=5.0)
    filtered_wind_sensor.direction = 90

    return {
        "gps": gps,
        "ais_ships": ais_ships,
        "global_path": global_path,
        "target_global_waypoint": global_path.waypoints[-1],
        "filtered_wind_sensor": filtered_wind_sensor,
        "planner": "rrtstar",
    }


def create_solved_ompl_path(path):
    ompl_path = mock.Mock()
    ompl_path.solved = True
    ompl_path.get_path.return_value = path
    return ompl_path


@pytest.mark.parametrize(
    "path, target_wp_index, boat_lat_lon, correct_heading, new_target_wp_index",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=1.0, longitude=1.0),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            HelperLatLon(latitude=0.0, longitude=-0.1),
            90.0,
            1,
        ),
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=1.0, longitude=1.0),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            HelperLatLon(latitude=0.1, longitude=0.0),
            180.0,
            1,
        ),
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=1.0, longitude=1.0),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            HelperLatLon(latitude=0.1, longitude=0.1),
            -135.0,
            1,
        ),
        (
            # Test: boat has reached waypoints[1], heading should be to waypoints[2].
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.2),
                    HelperLatLon(latitude=0.0, longitude=0.1),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            HelperLatLon(latitude=0.0, longitude=0.09999),
            -90.0,
            2,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index(
    path: Path,
    target_wp_index: int,
    boat_lat_lon: HelperLatLon,
    correct_heading: float,
    new_target_wp_index: int,
):
    calculated_answer = lp.LocalPath.calculate_desired_heading_and_wp_index(
        path, target_wp_index, boat_lat_lon
    )

    assert calculated_answer[0] == pytest.approx(correct_heading, abs=3e-1)
    assert calculated_answer[1] == new_target_wp_index


@pytest.mark.parametrize(
    "path, target_wp_index, boat_lat_lon, expected_exception",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.2),
                    HelperLatLon(latitude=0.0, longitude=0.1),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            0,
            HelperLatLon(latitude=0.0, longitude=0.09999),
            lp.PathNotFoundError,
        ),
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.2),
                    HelperLatLon(latitude=0.0, longitude=0.1),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            3,
            HelperLatLon(latitude=0.0, longitude=0.09999),
            lp.PathNotFoundError,
        ),
        (
            None,
            100,
            HelperLatLon(latitude=0.0, longitude=0.09999),
            lp.PathNotFoundError,
        ),
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.1),
                    HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            1,
            HelperLatLon(latitude=0.0, longitude=0.00001),
            IndexError,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index_exception(
    path: Path,
    target_wp_index: int,
    boat_lat_lon: HelperLatLon,
    expected_exception,
):
    with pytest.raises(expected_exception):
        lp.LocalPath.calculate_desired_heading_and_wp_index(
            path, target_wp_index, boat_lat_lon
        )


@pytest.mark.parametrize(
    "target_local_wp_index, reference_latlon, path, obstacles, result",
    [
        (
            1,
            HelperLatLon(latitude=10.0, longitude=10.0),
            Path(waypoints=[HelperLatLon(latitude=0.0, longitude=0.0)]),
            [],
            False,
        ),
        (
            1,
            HelperLatLon(latitude=10.0, longitude=10.0),
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.0),
                    HelperLatLon(latitude=2.5, longitude=2.5),
                    HelperLatLon(latitude=5.0, longitude=5.0),
                    HelperLatLon(latitude=7.5, longitude=7.5),
                    HelperLatLon(latitude=10.0, longitude=10.0),
                ]
            ),
            [],
            False,
        ),
        # Third test: obstacle at midpoint between (0,0) and (10,10)
        (
            1,
            REF,
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.0),
                    HelperLatLon(latitude=2.5, longitude=2.5),
                    HelperLatLon(latitude=5.0, longitude=5.0),
                    HelperLatLon(latitude=7.5, longitude=7.5),
                    HelperLatLon(latitude=10.0, longitude=10.0),
                ]
            ),
            [
                Obstacle(
                    reference=REF,
                    sailbot_position=HelperLatLon(latitude=0.0, longitude=0.0),
                    collision_zone=MultiPolygon(
                        cs.latlon_polygon_list_to_xy_polygon_list(
                            [
                                Polygon(
                                    [
                                        (5.0 - 0.1, 5.0 - 0.1),
                                        (5.0 - 0.1, 5.0 + 0.1),
                                        (5.0 + 0.1, 5.0 + 0.1),
                                        (5.0 + 0.1, 5.0 - 0.1),
                                    ]
                                )
                            ],
                            REF,
                        )
                    ),
                )  # type: ignore
            ],
            True,
        ),
        (
            1,
            REF,
            Path(
                waypoints=[
                    HelperLatLon(latitude=0.0, longitude=0.0),
                    HelperLatLon(latitude=2.5, longitude=2.5),
                    HelperLatLon(latitude=5.0, longitude=5.0),
                    HelperLatLon(latitude=7.5, longitude=7.5),
                    HelperLatLon(latitude=10.0, longitude=10.0),
                ]
            ),
            [
                Obstacle(
                    reference=REF,
                    sailbot_position=HelperLatLon(latitude=0.0, longitude=0.0),
                    collision_zone=MultiPolygon(
                        cs.latlon_polygon_list_to_xy_polygon_list(
                            [
                                Polygon(
                                    [
                                        (5.0 - 0.1, 5.5 - 0.1),
                                        (5.0 - 0.1, 5.5 + 0.1),
                                        (5.0 + 0.1, 5.5 + 0.1),
                                        (5.0 + 0.1, 5.5 - 0.1),
                                    ]
                                )
                            ],
                            REF,
                        )
                    ),
                )  # type: ignore
            ],
            False,
        ),
        (
            2,  # target_local_wp_index
            HelperLatLon(
                latitude=48.121368408203125, longitude=-137.02294921875
            ),  # reference_latlon
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.460002899169936, longitude=-125.0999984741211),
                    HelperLatLon(latitude=48.63827614874176, longitude=-127.23150716102047),
                    HelperLatLon(latitude=48.84684704393619, longitude=-129.93126000648425),
                    HelperLatLon(latitude=48.36208347752578, longitude=-132.19417995751115),
                    HelperLatLon(latitude=48.380718347238414, longitude=-130.4590043700099),
                    HelperLatLon(latitude=48.13918464289731, longitude=-132.7218106371612),
                    HelperLatLon(latitude=48.102202568706595, longitude=-134.96294341717459),
                    HelperLatLon(latitude=48.101040082965156, longitude=-134.47089971289932),
                    HelperLatLon(latitude=48.121368408203125, longitude=-137.02294921875),
                ]
            ),
            [
                # Obstacle 1
                Obstacle(
                    reference=HelperLatLon(
                        latitude=48.113521575927734, longitude=-135.67999267578125
                    ),
                    sailbot_position=cs.xy_to_latlon(
                        REF, cs.XY(x=878.268119211102, y=106.06146253551834)
                    ),
                    collision_zone=Polygon(
                        [
                            # already in XY
                            (99.6522679584995, -0.0980475891705976),
                            (215.7203671068329, 210.5454902622525),
                            (217.7322678553582, 209.4270603225763),
                            (100.09027045234744, -0.3415362916477365),
                            (99.6522679584995, -0.0980475891705976),
                        ]
                    ),
                ),  # type: ignore
                # Obstacle 2
                Obstacle(
                    reference=HelperLatLon(
                        latitude=48.197017669677734, longitude=-128.9371337890625
                    ),
                    sailbot_position=cs.xy_to_latlon(
                        REF, cs.XY(x=878.268119211102, y=106.06146253551834)
                    ),
                    collision_zone=Polygon(
                        [
                            (599.7313118870519, 39.76370963196999),
                            (603.2992311185736, 100.16316541173205),
                            (604.3994654559167, 100.09268751281846),
                            (600.231809086198, 39.73164919737777),
                            (599.7313118870519, 39.76370963196999),
                        ]
                    ),
                ),  # type: ignore
                # Obstacle 3
                Obstacle(
                    reference=HelperLatLon(
                        latitude=48.510276794433594, longitude=-126.15927124023438
                    ),
                    sailbot_position=cs.xy_to_latlon(
                        REF, cs.XY(x=878.268119211102, y=106.06146253551834)
                    ),
                    collision_zone=Polygon(
                        [
                            (799.7261319438433, 99.77070315844303),
                            (802.2764069094936, 130.16949421576257),
                            (803.1509561211411, 130.0906919029951),
                            (800.2260333486587, 99.72565893871158),
                            (799.7261319438433, 99.77070315844303),
                        ]
                    ),
                ),  # type: ignore
            ],
            True,  # Expected result: path is in collision zone
        ),
    ],
)
def test_in_collision_zone(target_local_wp_index, reference_latlon, path, obstacles, result):

    test_lp = create_test_local_path_for_in_collision_zone(
        target_local_wp_index,
        reference_latlon,
        path,
        obstacles
    )

    assert test_lp.in_collision_zone() == result


@pytest.mark.parametrize(
    "new_tw_data, previous_tw_data, result",
    [
        # Basic Test 1 (wind speed change is significant)
        (
            Wind(speed_kmph=10 + 2 * lp.WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=95.0),
            Wind(speed_kmph=10.0, dir_deg=95.0),
            True,
        ),
        # Boundaries
        (
            Wind(speed_kmph=10.0 + lp.WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=90.0),
            Wind(speed_kmph=10.0, dir_deg=90.0),
            True,
        ),
        (
            Wind(speed_kmph=10.0 - lp.WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=90.0),
            Wind(speed_kmph=10.0, dir_deg=90.0),
            True,
        ),
        # Basic Test 2 (wind dir change is significant)
        (
            Wind(speed_kmph=10.0, dir_deg=105.0 - 1.5 * lp.WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=12.0, dir_deg=105.0),
            True,
        ),
        # Boundaries
        (
            Wind(speed_kmph=10.0, dir_deg=80.0 + lp.WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=10.0, dir_deg=80.0),
            True,
        ),
        (
            Wind(speed_kmph=10.0, dir_deg=100.0 - lp.WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=10.0, dir_deg=100.0),
            True,
        ),
        # Basic Test 3 (No significant change)
        (
            Wind(
                speed_kmph=10.0 + 0.99 * lp.WIND_SPEED_CHANGE_THRESH_PROP * 10.0,
                dir_deg=99.0 - 0.9 * lp.WIND_DIRECTION_CHANGE_THRESH_DEG,
            ),
            Wind(speed_kmph=10.0, dir_deg=99.0),
            False,
        ),
        # Fourth Test: Circular nature of angles
        (Wind(speed_kmph=10.0, dir_deg=180), Wind(speed_kmph=10.0, dir_deg=-179.999), False),
        (
            Wind(speed_kmph=10.0, dir_deg=-178.0),
            Wind(speed_kmph=10.0, dir_deg=180 - lp.WIND_DIRECTION_CHANGE_THRESH_DEG + 2),
            True,
        ),
    ],
)
def test_is_significant_wind_change(new_tw_data, previous_tw_data, result):
    assert PATH.is_significant_wind_change(new_tw_data, previous_tw_data) == result


@pytest.mark.parametrize(
    "wind_readings, expected_length",
    [
        # Single reading
        ([Wind(speed_kmph=10.0, dir_deg=45.0)], 1),
        # Multiple readings below max
        ([Wind(speed_kmph=10.0 + i, dir_deg=45.0) for i in range(5)], 5),
        # Exactly at max length
        (
            [Wind(speed_kmph=10.0, dir_deg=45.0) for _ in range(lp.WIND_HISTORY_LEN)],
            lp.WIND_HISTORY_LEN,
        ),
        # Exceeding max length (should stay at max)
        (
            [Wind(speed_kmph=10.0, dir_deg=45.0) for _ in range(lp.WIND_HISTORY_LEN + 5)],
            lp.WIND_HISTORY_LEN,
        ),
    ],
)
def test_update_aw_history_length(wind_readings, expected_length):
    """Test that wind history respects max length constraint."""
    lps = lp.LocalPathState(
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
    )
    lps.aw_history.clear()

    for wind in wind_readings:
        lps.current_aw = wind
        lps.update_aw_history()

    assert len(lps.aw_history) == expected_length


def test_aw_history_fifo_order():
    """Test that oldest wind readings are removed first."""
    lps = lp.LocalPathState(
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
    )

    for i in range(lp.WIND_HISTORY_LEN + 3):
        lps.current_aw = Wind(speed_kmph=10.0 + i, dir_deg=45.0 + i)
        lps.update_aw_history()

    # Should contain the last WIND_HISTORY_LEN readings
    # First reading should have speed 10.0 + 3.0 (index 3 of original sequence)
    assert lps.aw_history[0].speed_kmph == 13.0
    # Last reading should have speed 10.0 + 2.0 + WIND_HISTORY_LEN
    assert lps.aw_history[-1].speed_kmph == 12.0 + lp.WIND_HISTORY_LEN


@pytest.mark.parametrize(
    "wind_readings, result",
    [
        # No readings (No average)
        (
            [],
            None,
        ),
        # Single reading (No average)
        (
            [Wind(speed_kmph=15.0, dir_deg=90.0)],
            None,
        ),
        # Same speed and direction
        (
            [Wind(speed_kmph=10.0, dir_deg=45.0) for _ in range(lp.WIND_HISTORY_LEN)],
            Wind(speed_kmph=10.0, dir_deg=45.0),
        ),
        # Different speeds, same direction
        (
            [Wind(speed_kmph=0, dir_deg=0) for _ in range(lp.WIND_HISTORY_LEN - 3)]
            + [
                Wind(speed_kmph=10.0, dir_deg=0.0),
                Wind(speed_kmph=20.0, dir_deg=0.0),
                Wind(speed_kmph=30.0, dir_deg=0.0),
            ],
            Wind(speed_kmph=60.0 / lp.WIND_HISTORY_LEN, dir_deg=0.0),
        ),
        # Same speed, opposite directions
        (
            [Wind(speed_kmph=10.0, dir_deg=0.0) for _ in range(lp.WIND_HISTORY_LEN // 2)]
            + [Wind(speed_kmph=10.0, dir_deg=180.0) for _ in range(lp.WIND_HISTORY_LEN // 2)],
            Wind(speed_kmph=10.0, dir_deg=90.0),
        ),
        # Circular mean of Angles, same speed
        (
            [Wind(speed_kmph=0, dir_deg=180) for _ in range(lp.WIND_HISTORY_LEN - 2)]
            + [
                Wind(speed_kmph=10.0, dir_deg=175.0),
                Wind(speed_kmph=10.0, dir_deg=-175.0),
            ],
            Wind(speed_kmph=20.0 / lp.WIND_HISTORY_LEN, dir_deg=180.0),
        ),
    ],
)
def test_calculate_aw_avg(wind_readings, result):
    """Test average wind calculation with basic cases."""
    lps = lp.LocalPathState(
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
    )

    for wind in wind_readings:
        lps.current_aw = wind
        lps.update_aw_history()

    avg = lps.aw_avg
    if result is None:
        assert avg is None
    else:
        assert avg.speed_kmph == result.speed_kmph
        assert avg.dir_deg == result.dir_deg


def test_aw_avg_not_set_before_full_history():
    """Test that wind_average isn't set until we have WIND_HISTORY_LEN wind readings."""
    lps = lp.LocalPathState(
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
    )
    lps.aw_history.clear()

    for _ in range(lp.WIND_HISTORY_LEN - 1):
        wind = Wind(speed_kmph=10.0, dir_deg=45.0)
        lps.current_aw = wind
        lps.update_aw_history()

    assert lps.aw_avg is None


def test_aw_avg_updates_with_new_readings():
    """Test that wind_average updates when new readings are added."""
    lps = lp.LocalPathState(
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
    )

    # Fill history with initial readings
    for _ in range(lp.WIND_HISTORY_LEN):
        lps.current_aw = Wind(speed_kmph=10.0, dir_deg=45.0)
        lps.update_aw_history()

    first_avg = lps.aw_avg

    # Add a different wind reading (oldest will be removed from deque)
    lps.current_aw = Wind(speed_kmph=30.0, dir_deg=45.0)
    lps.update_aw_history()
    second_avg = lps.aw_avg

    # Average should increase since we're replacing a 10.0 with a 30.0
    assert second_avg is not None
    assert first_avg is not None
    assert second_avg.speed_kmph > first_avg.speed_kmph


def test_LocalPathState_parameter_checking():
    with pytest.raises(ValueError):
        lps = (
            lp.LocalPathState(
                gps=None,
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

    with pytest.raises(ValueError):
        lps = (
            lp.LocalPathState(
                gps=GPS(),
                ais_ships=None,
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

    with pytest.raises(ValueError):
        lps = (
            lp.LocalPathState(
                gps=GPS(),
                ais_ships=AISShips(),
                global_path=Path(waypoints=[]),
                target_global_waypoint=HelperLatLon(),
                filtered_wind_sensor=WindSensor(),
                planner="rrtstar",
            ),
        )

    with pytest.raises(ValueError):
        lps = (
            lp.LocalPathState(
                gps=GPS(),
                ais_ships=AISShips(),
                global_path=None,
                target_global_waypoint=None,
                filtered_wind_sensor=WindSensor(),
                planner="rrtstar",
            ),
        )

    with pytest.raises(ValueError):
        lps = (
            lp.LocalPathState(
                gps=GPS(),
                ais_ships=AISShips(),
                global_path=Path(
                    waypoints=[
                        HelperLatLon(latitude=0.0, longitude=0.0),
                        HelperLatLon(latitude=1.0, longitude=1.0),
                    ]
                ),
                target_global_waypoint=HelperLatLon(latitude=1.0, longitude=1.0),
                filtered_wind_sensor=None,
                planner="rrtstar",
            ),
        )
    with pytest.raises(ValueError):
        lps = (  # noqa
            lp.LocalPathState(
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
                planner=None,
            ),
        )


@pytest.mark.parametrize(
    "elapsed,expected",
    [
        (lp.PATH_TTL_SEC + timedelta(seconds=1), True),
        (lp.PATH_TTL_SEC, True),
        (lp.PATH_TTL_SEC - timedelta(seconds=1), False),
    ],
)
def test_is_path_expired(elapsed, expected, basic_local_path_state):
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    basic_local_path_state.path_generated_time = datetime.now() - elapsed
    local_path.state = basic_local_path_state

    assert local_path.is_path_expired() == expected


@pytest.mark.parametrize(
    "scenario, received_new_global_waypoint, has_ompl_path, has_state, has_path,"
    " target_lp_wp_index, in_collision_zone, is_path_expired, expected_reason",
    [
        (
            "received_new_global_waypoint",
            True,
            True,
            True,
            True,
            1,
            False,
            False,
            "Received new global waypoint",
        ),
        (
            "ompl_path_is_none",
            False,
            False,
            True,
            True,
            1,
            False,
            False,
            "OMPL path is None",
        ),
        (
            "state_is_none",
            False,
            True,
            False,
            True,
            1,
            False,
            False,
            "State is None",
        ),
        (
            "path_is_none",
            False,
            True,
            True,
            False,
            1,
            False,
            False,
            "Path is None",
        ),
        (
            "path_intersects_collision_zone",
            False,
            True,
            True,
            True,
            1,
            True,
            False,
            "Path intersects collision zone",
        ),
        (
            "path_expired",
            False,
            True,
            True,
            True,
            1,
            False,
            True,
            "Path has expired (TTL exceeded)",
        ),
        (
            "target_waypoint_index_too_low",
            False,
            True,
            True,
            True,
            0,
            False,
            False,
            "Target waypoint index too low: 0",
        ),
        (
            "target_waypoint_index_out_of_bounds",
            False,
            True,
            True,
            True,
            2,
            False,
            False,
            "Target waypoint index out of bounds: 2 >= 2",
        ),
    ],
)
def test_update_if_needed_regenerates_path_when_path_must_change(
    scenario,  # noqa: ARG001
    received_new_global_waypoint,
    has_ompl_path,
    has_state,
    has_path,
    target_lp_wp_index,
    in_collision_zone,
    is_path_expired,
    expected_reason,
    basic_local_path_state,
):
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)

    old_path = Path(
        waypoints=[
            HelperLatLon(latitude=1.0, longitude=1.0),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    if has_ompl_path:
        local_path._ompl_path = create_solved_ompl_path(old_path)
    if has_state:
        local_path.state = basic_local_path_state
    if has_path:
        local_path.path = old_path

    new_path = Path(
        waypoints=[
            HelperLatLon(latitude=1.0, longitude=1.0),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    new_ompl_path = create_solved_ompl_path(new_path)
    inputs = create_update_if_needed_inputs()

    with (
        mock.patch.object(local_path, "in_collision_zone", return_value=in_collision_zone),
        mock.patch.object(local_path, "is_path_expired", return_value=is_path_expired),
        mock.patch.object(lp, "OMPLPath", return_value=new_ompl_path) as ompl_path_cls,
    ):
        desired_heading, new_target_lp_wp_index = local_path.update_if_needed(
            target_lp_wp_index=target_lp_wp_index,
            received_new_global_waypoint=received_new_global_waypoint,
            **inputs,
        )

    assert desired_heading == pytest.approx(90.0, abs=3e-1)
    assert new_target_lp_wp_index == 1
    assert local_path._ompl_path is new_ompl_path
    assert local_path.path is new_path
    assert local_path.state.global_path is inputs["global_path"]
    local_path._logger.debug.assert_any_call(f"Updating local path: {expected_reason}")
    ompl_path_cls.assert_called_once()


def test_update_if_needed_raises_when_path_generation_exceeds_retries():
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    inputs = create_update_if_needed_inputs()

    unsolved_ompl_path = mock.Mock()
    unsolved_ompl_path.solved = False

    with mock.patch.object(lp, "OMPLPath", return_value=unsolved_ompl_path) as ompl_path_cls:
        with pytest.raises(lp.PathNotFoundError, match="couldn't be solved"):
            local_path.update_if_needed(
                target_lp_wp_index=1,
                received_new_global_waypoint=True,
                **inputs,
            )

    assert ompl_path_cls.call_count == lp.MAX_OMPL_PATH_GEN_TRIES


def test_update_if_needed_raises_when_new_state_inputs_are_invalid():
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    inputs = create_update_if_needed_inputs()
    inputs["gps"] = None

    with pytest.raises(ValueError, match="gps must not be None"):
        local_path.update_if_needed(
            target_lp_wp_index=1,
            received_new_global_waypoint=True,
            **inputs,
        )


def test_update_if_needed_raises_when_solved_ompl_path_returns_invalid_path():
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    inputs = create_update_if_needed_inputs()

    ompl_path_with_invalid_path = create_solved_ompl_path(None)

    with mock.patch.object(lp, "OMPLPath", return_value=ompl_path_with_invalid_path):
        with pytest.raises(lp.PathNotFoundError, match="Path is None"):
            local_path.update_if_needed(
                target_lp_wp_index=1,
                received_new_global_waypoint=True,
                **inputs,
            )


def test_update_if_needed_raises_when_generated_path_has_no_next_waypoint():
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    inputs = create_update_if_needed_inputs()
    inputs["gps"].lat_lon = HelperLatLon(latitude=0.0, longitude=0.00001)

    path_with_reached_final_waypoint = Path(
        waypoints=[
            HelperLatLon(latitude=0.0, longitude=0.1),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    ompl_path_with_reached_final_waypoint = create_solved_ompl_path(
        path_with_reached_final_waypoint
    )

    with mock.patch.object(lp, "OMPLPath", return_value=ompl_path_with_reached_final_waypoint):
        with pytest.raises(IndexError, match="Must generate new path"):
            local_path.update_if_needed(
                target_lp_wp_index=1,
                received_new_global_waypoint=True,
                **inputs,
            )


def test_update_if_needed_propagates_update_state_errors_when_reusing_path(
    basic_local_path_state,
):
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)
    old_path = Path(
        waypoints=[
            HelperLatLon(latitude=1.0, longitude=1.0),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    local_path._ompl_path = create_solved_ompl_path(old_path)
    local_path.path = old_path
    local_path.state = basic_local_path_state
    local_path.state.path_generated_time = datetime.now()
    inputs = create_update_if_needed_inputs()
    inputs["gps"] = None

    with pytest.raises(ValueError, match="gps must not be None"):
        local_path.update_if_needed(
            target_lp_wp_index=1,
            received_new_global_waypoint=False,
            **inputs,
        )


def test_update_if_needed_no_change(
    basic_local_path_state,
):
    mock_parent_logger = mock.Mock()
    mock_parent_logger.get_child.return_value = mock.Mock()
    local_path = lp.LocalPath(parent_logger=mock_parent_logger)

    old_path = Path(
        waypoints=[
            HelperLatLon(latitude=1.0, longitude=1.0),
            HelperLatLon(latitude=0.0, longitude=0.0),
        ]
    )
    old_ompl_path = create_solved_ompl_path(old_path)
    local_path._ompl_path = old_ompl_path
    local_path.state = basic_local_path_state
    local_path.state.path_generated_time = datetime.now()
    local_path.path = old_path

    inputs = create_update_if_needed_inputs()
    original_global_path = local_path.state.global_path
    original_reference_latlon = local_path.state.reference_latlon

    with mock.patch.object(lp, "OMPLPath") as ompl_path_cls:
        desired_heading, new_target_lp_wp_index = local_path.update_if_needed(
            target_lp_wp_index=1,
            received_new_global_waypoint=False,
            **inputs,
        )

    assert desired_heading == pytest.approx(90.0, abs=3e-1)
    assert new_target_lp_wp_index == 1
    assert local_path._ompl_path is old_ompl_path
    assert local_path.path is old_path
    assert local_path.state.global_path is original_global_path
    assert local_path.state.reference_latlon is original_reference_latlon
    assert local_path.state.position is inputs["gps"].lat_lon
    assert local_path.state.speed == inputs["gps"].speed.speed
    assert local_path.state.heading == inputs["gps"].heading.heading
    assert local_path.state.current_aw.speed_kmph == inputs["filtered_wind_sensor"].speed.speed
    assert local_path.state.current_aw.dir_deg == inputs["filtered_wind_sensor"].direction
    # The apparent-wind average is not available until the history deque is full.
    assert local_path.state.aw_avg is None
    old_ompl_path.get_path.assert_called_once()
    ompl_path_cls.assert_not_called()
    local_path._logger.debug.assert_not_called()
