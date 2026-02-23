import pytest
from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from local_pathfinding.wind_coord_systems import Wind
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Polygon

import local_pathfinding.coord_systems as cs
import local_pathfinding.local_path as lp
from local_pathfinding.local_path import (WIND_SPEED_CHANGE_THRESH_PROP,
                                          WIND_DIRECTION_CHANGE_THRESH_DEG)
from local_pathfinding.obstacles import Obstacle

REF = HelperLatLon(latitude=10.0, longitude=10.0)

PATH = lp.LocalPath(parent_logger=RcutilsLogger())


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
    "new_tw_data, previous_tw_data, result",
    [
        # Basic Test 1 (wind speed change is significant)
        (
            Wind(speed_kmph=10 + 2 * WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=95.0),
            Wind(speed_kmph=10.0, dir_deg=95.0),
            True
        ),
        # Boundaries
        (
            Wind(speed_kmph=10.0 + WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=90.0),
            Wind(speed_kmph=10.0, dir_deg=90.0),
            True
        ),
        (
            Wind(speed_kmph=10.0 - WIND_SPEED_CHANGE_THRESH_PROP * 10.0, dir_deg=90.0),
            Wind(speed_kmph=10.0, dir_deg=90.0),
            True
        ),
        # Basic Test 2 (wind dir change is significant)
        (
            Wind(speed_kmph=10.0, dir_deg=105.0 - 1.5 * WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=12.0, dir_deg=105.0),
            True
        ),
        # Boundaries
        (
            Wind(speed_kmph=10.0, dir_deg=80.0 + WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=10.0, dir_deg=80.0),
            True
        ),
        (
            Wind(speed_kmph=10.0, dir_deg=100.0 - WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=10.0, dir_deg=100.0),
            True
        ),
        # Basic Test 3 (No significant change)
        (
            Wind(speed_kmph=10.0 + 0.99 * WIND_SPEED_CHANGE_THRESH_PROP * 10.0,
                 dir_deg=99.0 - 0.9 * WIND_DIRECTION_CHANGE_THRESH_DEG),
            Wind(speed_kmph=10.0, dir_deg=99.0),
            False
        ),
        # Fourth Test: Circular nature of angles
        (
            Wind(speed_kmph=10.0, dir_deg=180),
            Wind(speed_kmph=10.0, dir_deg=-179.999),
            False
        ),
        (
            Wind(speed_kmph=10.0, dir_deg=-178.0),
            Wind(speed_kmph=10.0, dir_deg=180 - WIND_DIRECTION_CHANGE_THRESH_DEG + 2),
            True
        ),
    ],
)
def test_significant_wind_change(new_tw_data,
                                 previous_tw_data,
                                 result):
    assert PATH.significant_wind_change(new_tw_data, previous_tw_data) == result


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
