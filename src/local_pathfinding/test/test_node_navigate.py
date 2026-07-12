from types import SimpleNamespace
from typing import cast

import custom_interfaces.msg as ci

from local_pathfinding.local_path import LocalPathInputs, PathNotFoundError
from local_pathfinding.node_navigate import GlobalPath, Sailbot


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def debug(self, *_args, **_kwargs) -> None:
        self.messages.append(("debug", str(_args[0]) if _args else ""))

    def info(self, *_args, **_kwargs) -> None:
        self.messages.append(("info", str(_args[0]) if _args else ""))

    def warning(self, *_args, **_kwargs) -> None:
        self.messages.append(("warning", str(_args[0]) if _args else ""))

    def error(self, *_args, **_kwargs) -> None:
        self.messages.append(("error", str(_args[0]) if _args else ""))

    def has_message(self, level: str, text: str) -> bool:
        return any(msg_level == level and text in msg for msg_level, msg in self.messages)


class FakeLocalPath:
    def __init__(self, update_calls: list[tuple[ci.HelperLatLon, bool, int]]) -> None:
        self.path = ci.Path(waypoints=[make_waypoint(48.0, -122.0)])
        self.update_calls = update_calls
        self.fail_next = False

    def update_if_needed(
        self,
        inputs: LocalPathInputs,
        target_lp_wp_index: int,
        received_new_global_waypoint: bool,
    ) -> tuple[float, int]:
        self.update_calls.append(
            (
                inputs.target_global_waypoint,
                received_new_global_waypoint,
                target_lp_wp_index,
            )
        )
        if self.fail_next:
            raise PathNotFoundError("unable to solve")
        return 123.0, target_lp_wp_index


def make_waypoint(latitude: float, longitude: float) -> ci.HelperLatLon:
    return ci.HelperLatLon(latitude=latitude, longitude=longitude)


def make_path(latitude_base: float, longitude_base: float, count: int = 3) -> ci.Path:
    return ci.Path(
        waypoints=[
            make_waypoint(latitude_base + index * 0.01, longitude_base - index * 0.01)
            for index in range(count)
        ]
    )


def waypoint_tuples(waypoints: list[ci.HelperLatLon]) -> list[tuple[float, float]]:
    return [(waypoint.latitude, waypoint.longitude) for waypoint in waypoints]


def make_sailbot_shell(gps_lat_lon: ci.HelperLatLon | None = None) -> Sailbot:
    sailbot = object.__new__(Sailbot)
    gps = None
    if gps_lat_lon is not None:
        gps = ci.GPS()
        gps.lat_lon = gps_lat_lon
    sailbot.gps = gps
    sailbot.received_new_global_path = False
    logger = FakeLogger()
    setattr(sailbot, "global_path_sub", SimpleNamespace(topic="global_path"))
    setattr(sailbot, "get_logger", lambda: logger)
    return sailbot


def require_gp(sailbot: Sailbot) -> GlobalPath:
    gp = sailbot.gp
    assert gp is not None
    return gp


def get_test_logger(sailbot: Sailbot) -> FakeLogger:
    return cast(FakeLogger, sailbot.get_logger())


def install_successful_global_path_write(sailbot: Sailbot) -> list[ci.Path]:
    write_calls: list[ci.Path] = []

    def write_global_path_to_file(path: ci.Path) -> None:
        write_calls.append(path)

    setattr(sailbot, "_write_global_path_to_file", write_global_path_to_file)
    return write_calls


def install_forbidden_persisted_load(sailbot: Sailbot, reason: str) -> None:
    def load_persisted_global_path() -> bool:
        raise AssertionError(reason)

    setattr(sailbot, "_load_persisted_global_path", load_persisted_global_path)


def install_failing_global_path_write(sailbot: Sailbot, error: OSError) -> list[ci.Path]:
    write_calls: list[ci.Path] = []

    def write_global_path_to_file(path: ci.Path) -> None:
        write_calls.append(path)
        raise error

    setattr(sailbot, "_write_global_path_to_file", write_global_path_to_file)
    return write_calls


def install_persisted_fallback_load(sailbot: Sailbot, fallback_path: ci.Path) -> list[bool]:
    load_calls: list[bool] = []

    def load_persisted_global_path() -> bool:
        load_calls.append(True)
        sailbot._set_gp(GlobalPath(waypoints=list(fallback_path.waypoints), index=2))
        return True

    setattr(sailbot, "_load_persisted_global_path", load_persisted_global_path)
    return load_calls


def install_local_path(
    sailbot: Sailbot,
) -> tuple[FakeLocalPath, list[tuple[ci.HelperLatLon, bool, int]]]:
    update_calls: list[tuple[ci.HelperLatLon, bool, int]] = []
    fake_local_path = FakeLocalPath(update_calls)
    setattr(sailbot, "local_path", fake_local_path)
    return fake_local_path, update_calls


def test_new_global_path_starts_at_last_waypoint() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    path = ci.Path(waypoints=waypoints)
    sailbot = make_sailbot_shell()

    gp = sailbot._create_gp(path, is_backup=False, is_new_global_path=True)

    assert gp is not None
    assert gp.index == len(waypoints) - 1
    assert gp.target_waypoint == waypoints[-1]
    assert not gp.is_backup


def test_persisted_global_path_resumes_one_waypoint_toward_destination() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    path = ci.Path(waypoints=waypoints)
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.11, -123.11))

    gp = sailbot._create_gp(path, is_backup=True)

    assert gp is not None
    assert gp.index == 0
    assert gp.target_waypoint == waypoints[0]
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


def test_global_path_switch_back_toggles_between_final_two_indices() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
    ]
    gp = GlobalPath(waypoints=waypoints, index=0)

    gp.trigger_switch_back()

    assert gp.switch_back_mode
    assert gp.index == 1
    assert gp.target_waypoint == waypoints[1]

    gp.do_switch_back()

    assert gp.index == 0
    assert gp.target_waypoint == waypoints[0]

    gp.do_switch_back()

    assert gp.index == 1
    assert gp.target_waypoint == waypoints[1]


def test_reaching_final_global_waypoint_triggers_switch_back() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
    ]
    sailbot = make_sailbot_shell(gps_lat_lon=waypoints[0])
    sailbot.gp = GlobalPath(waypoints=waypoints, index=0)
    sailbot.ais_ships = ci.AISShips()
    sailbot.filtered_wind_sensor = ci.WindSensor()
    sailbot.land_multi_polygon = None
    sailbot.target_lp_wp_index = 1
    sailbot.received_new_global_path = True
    _, update_calls = install_local_path(sailbot)

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 123.0
    assert sail
    assert require_gp(sailbot).switch_back_mode
    assert require_gp(sailbot).index == 1
    assert update_calls[-1] == (waypoints[1], True, 1)
    assert not sailbot.received_new_global_path

    gps = sailbot.gps
    assert gps is not None
    gps.lat_lon = waypoints[1]

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 123.0
    assert sail
    assert require_gp(sailbot).switch_back_mode
    assert require_gp(sailbot).index == 0
    assert update_calls[-1] == (waypoints[0], True, 1)
    assert not sailbot.received_new_global_path


def test_global_path_callback_success_replaces_gp_after_persisting() -> None:
    existing_path = make_path(49.0, -123.0)
    incoming_path = make_path(50.0, -124.0)
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(existing_path.waypoints), index=1)
    sailbot.received_new_global_path = False
    write_calls = install_successful_global_path_write(sailbot)
    install_forbidden_persisted_load(
        sailbot, "persisted fallback should not be loaded after successful write"
    )

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert write_calls == [incoming_path]
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(incoming_path.waypoints)
    assert gp.index == len(incoming_path.waypoints) - 1
    assert not gp.is_backup
    assert sailbot.received_new_global_path


def test_global_path_callback_new_path_clears_switch_back_mode() -> None:
    existing_path = make_path(49.0, -123.0)
    incoming_path = make_path(50.0, -124.0)
    sailbot = make_sailbot_shell()
    existing_gp = GlobalPath(waypoints=list(existing_path.waypoints), index=0)
    existing_gp.trigger_switch_back()
    sailbot.gp = existing_gp
    sailbot.received_new_global_path = False
    write_calls = install_successful_global_path_write(sailbot)
    install_forbidden_persisted_load(
        sailbot, "persisted fallback should not be loaded after successful write"
    )

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert write_calls == [incoming_path]
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(incoming_path.waypoints)
    assert gp.index == len(incoming_path.waypoints) - 1
    assert not gp.switch_back_mode
    assert sailbot.received_new_global_path


def test_global_path_callback_ignores_unchanged_active_main_path() -> None:
    incoming_path = make_path(50.0, -124.0)
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(incoming_path.waypoints), index=1, is_backup=False)
    sailbot.received_new_global_path = False
    install_forbidden_persisted_load(
        sailbot, "persisted fallback should not be loaded for unchanged active path"
    )

    def write_global_path_to_file(path: ci.Path) -> None:
        raise AssertionError("unchanged active path should not be persisted again")

    setattr(sailbot, "_write_global_path_to_file", write_global_path_to_file)

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(incoming_path.waypoints)
    assert gp.index == 1
    assert not gp.is_backup
    assert not sailbot.received_new_global_path
    assert get_test_logger(sailbot).has_message("debug", "Received unchanged global path")


def test_global_path_callback_ignores_active_main_path_with_float_jitter() -> None:
    active_path = ci.Path(
        waypoints=[
            make_waypoint(49.00000001, -123.00000001),
            make_waypoint(49.10000001, -123.10000001),
        ]
    )
    incoming_path = ci.Path(
        waypoints=[
            make_waypoint(49.00000002, -123.00000002),
            make_waypoint(49.10000002, -123.10000002),
        ]
    )
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(active_path.waypoints), index=1, is_backup=False)
    sailbot.received_new_global_path = False

    def write_global_path_to_file(path: ci.Path) -> None:
        raise AssertionError("numerically unchanged active path should not be persisted again")

    def load_persisted_global_path() -> bool:
        raise AssertionError("persisted fallback should not be loaded for unchanged active path")

    setattr(sailbot, "_write_global_path_to_file", write_global_path_to_file)
    setattr(sailbot, "_load_persisted_global_path", load_persisted_global_path)

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(active_path.waypoints)
    assert gp.index == 1
    assert not gp.is_backup
    assert not sailbot.received_new_global_path
    assert get_test_logger(sailbot).has_message("debug", "Received unchanged global path")


def test_global_path_callback_write_failure_uses_persisted_fallback() -> None:
    existing_path = make_path(49.0, -123.0)
    incoming_path = make_path(50.0, -124.0)
    fallback_path = make_path(51.0, -125.0)
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(existing_path.waypoints), index=1)
    sailbot.received_new_global_path = False
    write_calls = install_failing_global_path_write(sailbot, OSError("disk full"))
    load_calls = install_persisted_fallback_load(sailbot, fallback_path)

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert write_calls == [incoming_path]
    assert load_calls == [True]
    assert get_test_logger(sailbot).has_message("error", "Failed to persist global path")
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(fallback_path.waypoints)
    assert waypoint_tuples(gp.waypoints) != waypoint_tuples(incoming_path.waypoints)
    assert gp.index == len(fallback_path.waypoints) - 1
    assert sailbot.received_new_global_path


def test_global_path_callback_mixed_success_and_failure_sequence() -> None:
    path_a = make_path(49.0, -123.0)
    path_b = make_path(50.0, -124.0)
    path_c = make_path(51.0, -125.0)
    fallback_path = make_path(52.0, -126.0)
    path_d = make_path(53.0, -127.0)
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(path_a.waypoints), index=2)
    successful_writes: list[ci.Path] = []
    failing_paths = {id(path_c)}

    def write_global_path_to_file(path: ci.Path) -> None:
        if id(path) in failing_paths:
            raise OSError("disk full")
        successful_writes.append(path)

    setattr(sailbot, "_write_global_path_to_file", write_global_path_to_file)
    load_calls = install_persisted_fallback_load(sailbot, fallback_path)

    sailbot.global_path_callback(path_b)
    gp = require_gp(sailbot)
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(path_b.waypoints)
    assert gp.index == len(path_b.waypoints) - 1
    assert not gp.is_backup

    sailbot.global_path_callback(path_c)
    gp = require_gp(sailbot)
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(fallback_path.waypoints)
    assert waypoint_tuples(gp.waypoints) != waypoint_tuples(path_c.waypoints)
    assert gp.index == len(fallback_path.waypoints) - 1

    sailbot.global_path_callback(path_d)
    gp = require_gp(sailbot)
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(path_d.waypoints)
    assert gp.index == len(path_d.waypoints) - 1
    assert not gp.is_backup

    assert successful_writes == [path_b, path_d]
    assert load_calls == [True]


def test_new_global_path_signal_forces_replan_without_waypoint_advance() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    sailbot = make_sailbot_shell(gps_lat_lon=waypoints[0])
    sailbot.gp = GlobalPath(waypoints=waypoints, index=2)
    sailbot.ais_ships = ci.AISShips()
    sailbot.filtered_wind_sensor = ci.WindSensor()
    sailbot.land_multi_polygon = None
    sailbot.target_lp_wp_index = 1
    sailbot.received_new_global_path = True
    _, update_calls = install_local_path(sailbot)

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 123.0
    assert sail
    assert require_gp(sailbot).index == 2
    assert update_calls == [(waypoints[2], True, 1)]
    assert not sailbot.received_new_global_path


def test_global_waypoint_change_failure_keeps_retry_signal_without_accepting_waypoint() -> None:
    waypoints = [
        make_waypoint(49.0, -123.0),
        make_waypoint(49.1, -123.1),
        make_waypoint(49.2, -123.2),
    ]
    sailbot = make_sailbot_shell(gps_lat_lon=waypoints[2])
    sailbot.gp = GlobalPath(waypoints=waypoints, index=2)
    sailbot.ais_ships = ci.AISShips()
    sailbot.filtered_wind_sensor = ci.WindSensor()
    sailbot.land_multi_polygon = None
    sailbot.target_lp_wp_index = 1
    sailbot.received_new_global_path = False
    fake_local_path, update_calls = install_local_path(sailbot)

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 123.0
    assert sail
    assert require_gp(sailbot).index == 1
    assert update_calls[-1] == (waypoints[1], True, 1)
    assert not sailbot.received_new_global_path

    gps = sailbot.gps
    assert gps is not None
    gps.lat_lon = waypoints[1]
    fake_local_path.fail_next = True

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 0.0
    assert not sail
    assert require_gp(sailbot).index == 1
    assert update_calls[-1] == (waypoints[0], True, 1)
    assert sailbot.received_new_global_path
    assert fake_local_path.path.waypoints == []

    desired_heading, sail = sailbot.get_desired_heading()

    assert desired_heading == 0.0
    assert not sail
    assert require_gp(sailbot).index == 1
    assert update_calls[-1] == (waypoints[0], True, 1)
