from types import SimpleNamespace

import custom_interfaces.msg as ci

from local_pathfinding.node_navigate import GlobalPath, Sailbot


class FakeLogger:
    def debug(self, *_args, **_kwargs) -> None:
        pass

    def info(self, *_args, **_kwargs) -> None:
        pass

    def warning(self, *_args, **_kwargs) -> None:
        pass


def make_waypoint(latitude: float, longitude: float) -> ci.HelperLatLon:
    return ci.HelperLatLon(latitude=latitude, longitude=longitude)


def make_sailbot_shell(gps_lat_lon: ci.HelperLatLon | None = None) -> Sailbot:
    sailbot = object.__new__(Sailbot)
    sailbot.gps = SimpleNamespace(lat_lon=gps_lat_lon) if gps_lat_lon is not None else None
    sailbot.get_logger = lambda: FakeLogger()
    return sailbot


def test_main_global_path_starts_at_last_waypoint() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    path = ci.Path(waypoints=waypoints)
    sailbot = make_sailbot_shell()

    gp = sailbot._create_gp(path, is_backup=False)

    assert gp is not None
    assert gp.index == len(waypoints) - 1
    assert gp.target_waypoint == waypoints[-1]
    assert not gp.is_backup


def test_backup_global_path_starts_at_closest_gps_waypoint() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    path = ci.Path(waypoints=waypoints)
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.11, -123.11))

    gp = sailbot._create_gp(path, is_backup=True)

    assert gp is not None
    assert gp.index == 1
    assert gp.target_waypoint == waypoints[1]
    assert gp.is_backup


def test_backup_global_path_is_not_created_without_gps() -> None:
    path = ci.Path(waypoints=[make_waypoint(49.0, -123.0)])
    sailbot = make_sailbot_shell()

    gp = sailbot._create_gp(path, is_backup=True)

    assert gp is None


def test_global_path_advance_decrements_index_until_exhausted() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    gp = GlobalPath(waypoints=waypoints, index=2)

    assert gp.advance_waypoint()
    assert gp.index == 1
    assert gp.target_waypoint == waypoints[1]

    assert gp.advance_waypoint()
    assert gp.index == 0
    assert gp.target_waypoint == waypoints[0]

    assert not gp.advance_waypoint()
    assert gp.index == -1
    assert gp.target_waypoint is None


def test_reaching_final_global_waypoint_disables_sail() -> None:
    final_waypoint = make_waypoint(49.0, -123.0)
    sailbot = make_sailbot_shell(gps_lat_lon=final_waypoint)
    sailbot.gp = GlobalPath(waypoints=[final_waypoint], index=0)
    sailbot.local_path = SimpleNamespace(path=None)
    sailbot.received_new_global_waypoint = False

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 0.0
    assert not sail
    assert sailbot.gp.index == -1
    assert sailbot.local_path.path.waypoints == []
