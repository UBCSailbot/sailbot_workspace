from custom_interfaces.msg import GPS, AISShips, HelperLatLon, Path, WindSensor
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.local_path as local_path

PATH = local_path.LocalPath(parent_logger=RcutilsLogger())


def test_LocalPath_update_if_needed():

    PATH.update_if_needed(
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
    )
    assert PATH.path is not None, "waypoints is not initialized"
    assert len(PATH.path.waypoints) > 1, "waypoints length <= 1"
