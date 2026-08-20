from types import SimpleNamespace
from typing import cast

import pytest

import custom_interfaces.msg as ci
import local_pathfinding.local_path as lp
import local_pathfinding.obstacles as ob
from local_pathfinding.constants import (
    AIS_SHIPS_UNAVAILABLE,
    DESIRED_HEADING_UNAVAILABLE,
    GPS_UNAVAILABLE,
    HEADING_UNAVAILABLE,
    WIND_SENSOR_UNAVAILABLE,
)
from local_pathfinding.local_path import LocalPathInputs, PathNotFoundError
from local_pathfinding.node_navigate import AIS_SHIP_TIMEOUT_SEC, GlobalPath, Sailbot


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
        self.last_inputs: LocalPathInputs | None = None

    def update_if_needed(
        self,
        inputs: LocalPathInputs,
        target_lp_wp_index: int,
        received_new_global_waypoint: bool,
    ) -> tuple[float, int]:
        self.last_inputs = inputs
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
    sailbot.heading = ci.HelperHeading(heading=45.0) if gps is not None else None
    sailbot.received_new_global_path = False
    setattr(sailbot, "_tracked_ais_ships", None)
    # __init__ is bypassed here; seed the respawn self-heal tracking the callbacks touch (#1065).
    sailbot._sub_received = {
        "gps": False,
        "rudder": False,
        "filtered_wind_sensor": False,
        "ais_ships": False,
    }
    logger = FakeLogger()
    setattr(sailbot, "global_path_sub", SimpleNamespace(topic="global_path"))
    setattr(sailbot, "ais_ships_sub", SimpleNamespace(topic="ais_ships"))
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


def test_heading_callback_accepts_valid_rudder_heading() -> None:
    sailbot = make_sailbot_shell()
    sailbot.heading_sub = SimpleNamespace(topic="rudder")
    sailbot.heading_timeout_start_sec = 1.0
    setattr(sailbot, "_now_sec", lambda: 10.0)
    heading = ci.HelperHeading(heading=180.0)

    sailbot.heading_callback(heading)

    assert sailbot.heading is heading
    assert sailbot.heading_timeout_start_sec == 10.0


@pytest.mark.parametrize("heading_deg", [float("nan"), float("inf"), -180.0, 180.1])
def test_heading_callback_rejects_invalid_rudder_heading(heading_deg: float) -> None:
    sailbot = make_sailbot_shell()
    sailbot.heading_sub = SimpleNamespace(topic="rudder")
    previous_heading = ci.HelperHeading(heading=12.0)
    sailbot.heading = previous_heading
    sailbot.heading_timeout_start_sec = 1.0
    setattr(sailbot, "_now_sec", lambda: 10.0)

    sailbot.heading_callback(ci.HelperHeading(heading=heading_deg))

    assert sailbot.heading is previous_heading
    assert sailbot.heading_timeout_start_sec == 1.0
    assert get_test_logger(sailbot).has_message("warning", "Ignoring invalid heading")


def test_all_subs_active_requires_rudder_heading() -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    sailbot._tracked_ais_ships = {}
    sailbot.gp = GlobalPath(waypoints=list(make_path(49.0, -123.0).waypoints), index=2)
    sailbot.filtered_wind_sensor = ci.WindSensor()

    assert sailbot._all_subs_active()

    sailbot.heading = None

    assert not sailbot._all_subs_active()


def test_all_subs_active_requires_an_ais_message() -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    sailbot.gp = GlobalPath(waypoints=list(make_path(49.0, -123.0).waypoints), index=2)
    sailbot.filtered_wind_sensor = ci.WindSensor()

    assert not sailbot._all_subs_active()

    deliver_ais(sailbot, [], now_sec=0.0)

    assert sailbot._all_subs_active()


@pytest.mark.parametrize(
    ("now_sec", "gps_timestamp", "heading_timestamp", "expected"),
    [
        (121.0, 121.0, 121.0, False),
        (120.0, 0.0, 0.0, False),
        (121.0, 0.0, 121.0, True),
        (121.0, 121.0, 0.0, True),
        (121.0, 0.0, 0.0, True),
    ],
)
def test_timed_out_inputs(
    now_sec: float,
    gps_timestamp: float,
    heading_timestamp: float,
    expected: bool,
) -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    sailbot.gps_timeout_start_sec = gps_timestamp
    sailbot.heading_timeout_start_sec = heading_timestamp
    setattr(sailbot, "_now_sec", lambda: now_sec)

    assert sailbot._timed_out_inputs() is expected
    if heading_timestamp == 0.0 and now_sec - heading_timestamp > 120.0:
        assert sailbot.heading is not None
        assert sailbot.heading.heading == HEADING_UNAVAILABLE


@pytest.mark.parametrize(
    ("gps_timestamp", "heading_timestamp"), [(0.0, 121.0), (121.0, 0.0)]
)
def test_desired_heading_callback_disables_sail_for_timed_out_inputs(
    gps_timestamp: float, heading_timestamp: float, monkeypatch
) -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    sailbot.gp = GlobalPath(waypoints=list(make_path(49.0, -123.0).waypoints), index=2)
    sailbot.gps_timeout_start_sec = gps_timestamp
    sailbot.heading_timeout_start_sec = heading_timestamp
    setattr(sailbot, "_now_sec", lambda: 121.0)
    published: list[ci.DesiredHeading] = []
    sailbot.desired_heading_pub = SimpleNamespace(
        topic="desired_heading", publish=published.append
    )
    local_path_publications: list[bool] = []
    monkeypatch.setattr(
        sailbot,
        "publish_local_path_data",
        local_path_publications.append,
    )

    sailbot.desired_heading_callback()

    assert len(published) == 1
    assert not published[0].sail
    assert published[0].heading.heading == 0.0
    assert local_path_publications == [False]
    assert get_test_logger(sailbot).has_message("warning", "GPS or rudder data")


def test_publish_local_path_data_always_publishes_with_typed_defaults() -> None:
    sailbot = object.__new__(Sailbot)
    sailbot.gps = None
    sailbot.heading = None
    sailbot.filtered_wind_sensor = None
    sailbot._tracked_ais_ships = None
    setattr(sailbot, "_now_sec", lambda: 0.0)
    setattr(sailbot, "get_logger", lambda: FakeLogger())
    sailbot.gp = None
    sailbot.desired_heading = None
    setattr(
        sailbot,
        "local_path",
        SimpleNamespace(
            path=None,
            state=None,
            last_replan_reason="",
            last_remaining_waypoints=0,
        ),
    )
    published: list[ci.LPathData] = []
    sailbot.lpath_data_pub = SimpleNamespace(publish=published.append)

    sailbot.publish_local_path_data(sail=False)

    assert len(published) == 1
    assert published[0].heading.heading == HEADING_UNAVAILABLE
    assert published[0].gps == GPS_UNAVAILABLE
    assert published[0].gps.lat_lon.latitude == 91.0
    assert published[0].gps.lat_lon.longitude == 181.0
    assert published[0].gps.speed.speed == -1.0
    assert published[0].gps.heading.heading == HEADING_UNAVAILABLE
    assert published[0].filtered_wind_sensor == WIND_SENSOR_UNAVAILABLE
    assert published[0].ais_ships == AIS_SHIPS_UNAVAILABLE
    assert published[0].desired_heading == DESIRED_HEADING_UNAVAILABLE


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


def test_single_waypoint_global_path_is_not_created() -> None:
    path = ci.Path(waypoints=[make_waypoint(49.0, -123.0)])
    sailbot = make_sailbot_shell()

    gp = sailbot._create_gp(path, is_backup=False, is_new_global_path=True)

    assert gp is None


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
    sailbot._tracked_ais_ships = {}
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


def test_global_path_callback_rejects_single_waypoint_before_persisting() -> None:
    existing_path = make_path(49.0, -123.0)
    incoming_path = ci.Path(waypoints=[make_waypoint(50.0, -124.0)])
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(existing_path.waypoints), index=1)
    write_calls = install_successful_global_path_write(sailbot)
    install_forbidden_persisted_load(
        sailbot, "persisted fallback should not replace an existing active path"
    )

    sailbot.global_path_callback(incoming_path)

    gp = require_gp(sailbot)
    assert write_calls == []
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(existing_path.waypoints)
    assert get_test_logger(sailbot).has_message("warning", "fewer than two waypoints")


def test_global_path_callback_defensively_rejects_none_waypoints() -> None:
    existing_path = make_path(49.0, -123.0)
    malformed_path = cast(ci.Path, SimpleNamespace(waypoints=None))
    sailbot = make_sailbot_shell()
    sailbot.gp = GlobalPath(waypoints=list(existing_path.waypoints), index=1)
    write_calls = install_successful_global_path_write(sailbot)
    install_forbidden_persisted_load(
        sailbot, "persisted fallback should not replace an existing active path"
    )

    sailbot.global_path_callback(malformed_path)

    gp = require_gp(sailbot)
    assert write_calls == []
    assert waypoint_tuples(gp.waypoints) == waypoint_tuples(existing_path.waypoints)
    assert get_test_logger(sailbot).has_message("warning", "fewer than two waypoints")


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
    sailbot._tracked_ais_ships = {}
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
    sailbot._tracked_ais_ships = {}
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


# ========================= AIS SHIP TRACKING =========================
def make_ais_ship(
    ship_id: int,
    latitude: float = 49.30,
    longitude: float = -123.20,
) -> ci.HelperAISShip:
    return ci.HelperAISShip(
        id=ship_id,
        lat_lon=make_waypoint(latitude, longitude),
        cog=ci.HelperHeading(heading=30.0),
        sog=ci.HelperSpeed(speed=20.0),
        width=ci.HelperDimension(dimension=20.0),
        length=ci.HelperDimension(dimension=100.0),
        rot=ci.HelperROT(rot=0),
    )


def deliver_ais(sailbot: Sailbot, ships: list[ci.HelperAISShip], now_sec: float) -> None:
    """Deliver an AIS message to sailbot as if the clock read now_sec."""
    setattr(sailbot, "_now_sec", lambda: now_sec)
    sailbot.ais_ships_callback(ci.AISShips(ships=ships))


def tracked_ids(sailbot: Sailbot) -> list[int]:
    return sorted(ship.id for ship in sailbot._rebuild_ais_snapshot().ships)


def test_partial_ais_message_retains_omitted_ships() -> None:
    sailbot = make_sailbot_shell()
    deliver_ais(sailbot, [make_ais_ship(1), make_ais_ship(2)], now_sec=0.0)
    assert tracked_ids(sailbot) == [1, 2]

    deliver_ais(sailbot, [make_ais_ship(1, latitude=49.31)], now_sec=6.0)

    assert tracked_ids(sailbot) == [1, 2]
    ais_ships = sailbot._rebuild_ais_snapshot()
    refreshed = next(ship for ship in ais_ships.ships if ship.id == 1)
    assert refreshed.lat_lon.latitude == 49.31


def test_empty_ais_message_retains_tracked_ships() -> None:
    sailbot = make_sailbot_shell()
    deliver_ais(sailbot, [make_ais_ship(1), make_ais_ship(2)], now_sec=0.0)

    deliver_ais(sailbot, [], now_sec=15.0)

    assert tracked_ids(sailbot) == [1, 2]


def test_tracked_ais_ship_expires_after_timeout() -> None:
    sailbot = make_sailbot_shell()
    deliver_ais(sailbot, [make_ais_ship(1), make_ais_ship(2)], now_sec=0.0)

    deliver_ais(sailbot, [make_ais_ship(1)], now_sec=AIS_SHIP_TIMEOUT_SEC + 1.0)

    assert tracked_ids(sailbot) == [1]
    assert get_test_logger(sailbot).has_message("warning", "Dropping 1 AIS ship(s) [2]")


def test_ais_refresh_resets_the_timeout() -> None:
    sailbot = make_sailbot_shell()
    deliver_ais(sailbot, [make_ais_ship(1)], now_sec=0.0)
    deliver_ais(sailbot, [make_ais_ship(1)], now_sec=AIS_SHIP_TIMEOUT_SEC - 100.0)

    # Well past the timeout measured from the first message, but not from the refresh.
    deliver_ais(sailbot, [], now_sec=AIS_SHIP_TIMEOUT_SEC + 300.0)

    assert tracked_ids(sailbot) == [1]


def test_unavailable_sentinel_is_never_tracked() -> None:
    sailbot = make_sailbot_shell()

    deliver_ais(sailbot, list(AIS_SHIPS_UNAVAILABLE.ships), now_sec=0.0)

    assert tracked_ids(sailbot) == []
    assert sailbot._tracked_ais_ships == {}


def test_expired_ais_snapshot_is_empty_rather_than_unavailable() -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    deliver_ais(sailbot, [make_ais_ship(1)], now_sec=0.0)

    deliver_ais(sailbot, [], now_sec=AIS_SHIP_TIMEOUT_SEC + 1.0)

    # AIS is working and nothing is in range; sail must not be disabled for a missing input.
    assert tracked_ids(sailbot) == []
    sailbot.gp = GlobalPath(waypoints=list(make_path(49.0, -123.0).waypoints), index=2)
    sailbot.filtered_wind_sensor = ci.WindSensor()
    assert sailbot._all_subs_active()


def test_tracked_ais_ships_expire_without_a_new_message() -> None:
    sailbot = make_sailbot_shell(gps_lat_lon=make_waypoint(49.0, -123.0))
    deliver_ais(sailbot, [make_ais_ship(1)], now_sec=0.0)

    now_sec = AIS_SHIP_TIMEOUT_SEC + 1.0
    setattr(sailbot, "_now_sec", lambda: now_sec)

    assert tracked_ids(sailbot) == []


def make_local_path_state(ais_ships: ci.AISShips) -> lp.LocalPathState:
    global_path = make_path(49.0, -123.0)
    return lp.LocalPathState(
        gps=ci.GPS(
            lat_lon=make_waypoint(49.28, -123.18),
            speed=ci.HelperSpeed(speed=15.0),
            heading=ci.HelperHeading(heading=180.0),
        ),
        heading=ci.HelperHeading(heading=180.0),
        ais_ships=ais_ships,
        global_path=global_path,
        target_global_waypoint=global_path.waypoints[-1],
        filtered_wind_sensor=ci.WindSensor(speed=ci.HelperSpeed(speed=5.0), direction=90),
        wind_tracker=lp.WindTracker(),
    )


def refresh_state(state: lp.LocalPathState, ais_ships: ci.AISShips) -> None:
    state.update_state(
        gps=ci.GPS(
            lat_lon=state.position,
            speed=ci.HelperSpeed(speed=state.speed),
            heading=ci.HelperHeading(heading=state.heading),
        ),
        heading=ci.HelperHeading(heading=state.heading),
        ais_ships=ais_ships,
        filtered_wind_sensor=ci.WindSensor(speed=ci.HelperSpeed(speed=5.0), direction=90),
    )
    state.update_obstacles()


def obstacle_boat_ids(state: lp.LocalPathState) -> list[int]:
    return sorted(
        obstacle.ais_ship.id for obstacle in state.obstacles if isinstance(obstacle, ob.Boat)
    )


def test_tracked_ais_ships_survive_partial_updates_end_to_end() -> None:
    """Boats omitted by a partial AIS message stay in the planner's obstacle set.

    Walks the whole consumer chain the bug broke: ais_ships_callback -> LocalPathState ->
    update_obstacles, following the timings in test_plans/ais_partial_updates.yaml.
    """
    sailbot = make_sailbot_shell()
    deliver_ais(
        sailbot,
        [make_ais_ship(1), make_ais_ship(2, latitude=49.25, longitude=-123.15)],
        now_sec=0.0,
    )

    state = make_local_path_state(sailbot._rebuild_ais_snapshot())
    state.obstacles = ob.update_boat_obstacles(
        obstacles=[],
        reference=state.reference_latlon,
        sailbot_position=state.position,
        sailbot_speed=state.speed,
        ais_ships=state.ais_ships,
    )
    assert obstacle_boat_ids(state) == [1, 2]

    # t=6s: a message carrying only boat 1. Boat 2 must not vanish from collision detection.
    deliver_ais(sailbot, [make_ais_ship(1, latitude=49.31)], now_sec=6.0)
    refresh_state(state, sailbot._rebuild_ais_snapshot())
    assert obstacle_boat_ids(state) == [1, 2]

    # t=15s: an empty message. Both boats must still be there.
    deliver_ais(sailbot, [], now_sec=15.0)
    refresh_state(state, sailbot._rebuild_ais_snapshot())
    assert obstacle_boat_ids(state) == [1, 2]

    # Only the timeout removes them, and each boat ages from its own last refresh: boat 2 was
    # last seen at t=0 while boat 1 was refreshed at t=6, so boat 2 goes first.
    deliver_ais(sailbot, [], now_sec=AIS_SHIP_TIMEOUT_SEC + 1.0)
    refresh_state(state, sailbot._rebuild_ais_snapshot())
    assert obstacle_boat_ids(state) == [1]

    deliver_ais(sailbot, [], now_sec=AIS_SHIP_TIMEOUT_SEC + 100.0)
    refresh_state(state, sailbot._rebuild_ais_snapshot())
    assert obstacle_boat_ids(state) == []
