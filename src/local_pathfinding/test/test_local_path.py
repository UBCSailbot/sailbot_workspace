import pytest
from custom_interfaces.msg import (
    GPS, AISShips, HelperLatLon, HelperHeading, HelperSpeed,
    HelperAISShip, HelperDimension, HelperROT, Path, WindSensor
)
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.local_path as lp
import custom_interfaces.msg as ci
from local_pathfinding.obstacles import Obstacle
from local_pathfinding.ompl_path import create_mock

REF = HelperLatLon(latitude=10.0, longitude=10.0)

PATH = lp.LocalPath(parent_logger=RcutilsLogger())


@pytest.mark.parametrize(
    "x, y, x_normalized, y_normalized",
    [
        (2.0, 4.0, 0.5, 1.0),
        (0.2, 0.4, 0.2, 0.4),
        (3.0, 3.0, 1.0, 1.0),
        (0.0, 5.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 0.0),
        (0.5, 2.0, 0.25, 1.0),
    ]
)
def test_normalize_cost_pair(x, y, x_normalized, y_normalized):
    assert x_normalized == lp.normalize_cost_pair(x, y)[0]
    assert y_normalized == lp.normalize_cost_pair(x, y)[1]


@pytest.mark.parametrize(
    "path, waypoint_index, boat_lat_lon, correct_heading, new_wp_index",
    [
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=-0.1),
            90.0,
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.0),
            180.0,
            0,
        ),
        (
            ci.Path(waypoints=[ci.HelperLatLon(latitude=0.0, longitude=0.0)]),
            0,
            ci.HelperLatLon(latitude=0.1, longitude=0.1),
            -135.0,
            0,
        ),
        (
            # Test: boat has reached waypoints[0], heading should be to waypoints[1].
            ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=0.0, longitude=0.1),
                    ci.HelperLatLon(latitude=0.0, longitude=0.0),
                ]
            ),
            0,
            ci.HelperLatLon(latitude=0.0, longitude=0.09999),
            -90.0,
            1,
        ),
    ],
)
def test_calculate_desired_heading_and_waypoint_index(
    path: ci.Path,
    waypoint_index: int,
    boat_lat_lon: ci.HelperLatLon,
    correct_heading: float,
    new_wp_index: int,
):
    calculated_answer = PATH.calculate_desired_heading_and_waypoint_index(
        path, waypoint_index, boat_lat_lon
    )
    assert calculated_answer[0] == pytest.approx(correct_heading, abs=3e-1)
    assert calculated_answer[1] == new_wp_index


@pytest.mark.parametrize(
    "local_wp_index, reference_latlon, path, obstacles, result",
    [
        (
            0,
            HelperLatLon(latitude=10.0, longitude=10.0),
            Path(waypoints=[HelperLatLon(latitude=0.0, longitude=0.0)]),
            [],
            False,
        ),
        (
            0,
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
            0,
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
            0,
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
            1,  # local_wp_index
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
def test_in_collision_zone(local_wp_index, reference_latlon, path, obstacles, result):
    assert PATH.in_collision_zone(local_wp_index, reference_latlon, path, obstacles) == result


@pytest.mark.parametrize(
    '''
    gps, ais_ships, local_waypoint_index, received_new_global_waypoint,
    old_path, result_index, new_path
    ''',
    [
        (   # Old path is optimal, do not switch to new path
            GPS(lat_lon=HelperLatLon(latitude=0.0, longitude=0.0)),
            AISShips(),
            1,
            False,
            [
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=3.0, longitude=3.0)
            ],
            1,
            False
        ),
        (   # New global waypoint received, switch to new path
            GPS(lat_lon=HelperLatLon(latitude=0.0, longitude=0.0)),
            AISShips(),
            1,
            True,
            [
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=3.0, longitude=3.0)
            ],
            1,
            True
        ),
        (   # AISShip in path, switch to new path
            GPS(lat_lon=HelperLatLon(latitude=0.0, longitude=0.0)),
            AISShips(
                ships=[
                    HelperAISShip(
                        id=1,
                        lat_lon=HelperLatLon(latitude=1.0, longitude=1.0),
                        cog=HelperHeading(heading=45.0),
                        sog=HelperSpeed(speed=0.0),
                        width=HelperDimension(dimension=50.0),
                        length=HelperDimension(dimension=50.0),
                        rot=HelperROT(rot=0),
                    )
                ]
            ),
            1,
            True,
            [
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=1.0, longitude=1.0),
                HelperLatLon(latitude=3.0, longitude=3.0)
            ],
            1,
            True
        ),
        (   # Old path not optimal, switch to new path
            GPS(lat_lon=HelperLatLon(latitude=0.0, longitude=0.0)),
            AISShips(),
            1,
            False,
            [
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=5.0, longitude=-5.0),
                HelperLatLon(latitude=3.0, longitude=3.0)
            ],
            1,
            True
        )
    ],
)
def test_update_if_needed(
                        gps, ais_ships, local_waypoint_index, received_new_global_waypoint,
                        old_path, result_index, new_path
                        ):

    mock_ompl_path = create_mock(old_path, RcutilsLogger())
    PATH._ompl_path = mock_ompl_path

    current_path = mock_ompl_path.get_path()

    old_heading, _ = PATH.calculate_desired_heading_and_waypoint_index(
        current_path,
        local_waypoint_index,
        gps.lat_lon,
    )

    heading, index, update = PATH.update_if_needed(
        gps,
        ais_ships,
        Path(
            waypoints=[
                HelperLatLon(latitude=0.0, longitude=0.0),
                HelperLatLon(latitude=3.0, longitude=3.0),
            ]
        ),
        local_waypoint_index,
        received_new_global_waypoint,
        HelperLatLon(latitude=3.0, longitude=3.0),
        WindSensor(),
        "rrtstar",
        None,
    )

    if new_path:
        if not received_new_global_waypoint:
            assert abs(heading - old_heading) > 3
        assert update
    else:
        assert heading == pytest.approx(old_heading, abs=3)
        assert not update

    assert index == result_index


@pytest.mark.parametrize(
    '''
    heading, heading_old_path, heading_new_path, result
    ''',
    [
        (
            0.0,
            1.0,
            -1.1,
            0.5454545454545455
        ),
        (
            0.0,
            1.1,
            1.0,
            0.6
        ),
        (
            50.0,
            49.8,
            50.1,
            0.12
        ),
        (
            180.0,
            -179.0,
            178.0,
            0.3
        ),
        (
            0.0,
            1.0,
            -1.0,
            0.6
        ),
        (
            1.0,
            1.0,
            1.0,
            0.0
        )
    ]
)
def test_calculate_metric(heading, heading_old_path, heading_new_path, result):
    assert result == pytest.approx(PATH.calculate_metric(heading, heading_old_path, 0.0,
                                                         heading_new_path, 0.0), abs=1e-9)


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
