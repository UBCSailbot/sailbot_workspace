from pathlib import Path

import pytest
from shapely.geometry import MultiPolygon
from test_plans.test_plan import (
    AisEvent,
    GlobalPathEvent,
    GpsEvent,
    TestPlan,
    WindEvent,
)

import custom_interfaces.msg as ci

BASIC_PLAN = "basic.yaml"


@pytest.fixture(autouse=True)
def _reset_test_plan_singleton():
    """Reset the TestPlan singleton so each test loads its own YAML from scratch."""
    TestPlan._instance = None
    yield
    TestPlan._instance = None


@pytest.fixture
def make_plan(tmp_path: Path):
    """Write a YAML string to a temp file and return a fresh TestPlan for it."""
    def _make(content: str) -> TestPlan:
        path = tmp_path / "plan.yaml"
        path.write_text(content)
        return TestPlan(str(path))
    return _make


def test_instantiate_test_plan_object():
    plan = TestPlan(BASIC_PLAN)
    assert isinstance(plan.land, MultiPolygon)
    assert len(plan.land.geoms) > 0
    assert len(plan.ais) > 0
    assert isinstance(plan.gps, ci.GPS)
    assert isinstance(plan.global_path, ci.Path)
    assert isinstance(plan.tw_dir_deg, int)
    assert isinstance(plan.tw_speed_kmph, float)


def test_legacy_plan_has_empty_events():
    plan = TestPlan(BASIC_PLAN)
    assert plan.wind_events == ()
    assert plan.gps_events == ()
    assert plan.ais_events == ()
    assert plan.global_path_events == ()


def test_missing_events_block_gives_empty_lists(make_plan):
    plan = make_plan("tw_speed_kmph: 10.0\ntw_dir_deg: 0\n")
    assert plan.wind_events == ()
    assert plan.gps_events == ()
    assert plan.ais_events == ()
    assert plan.global_path_events == ()


def test_wind_events_parsed(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_wind_sensor:\n"
        "    - {timestamp: 0.0, direction_deg: 90, speed_kmph: 12.0}\n"
        "    - {timestamp: 5.5, direction_deg: 110, speed_kmph: 18.0}\n"
    )
    assert plan.wind_events == (
        WindEvent(timestamp=0.0, direction_deg=90, speed_kmph=12.0),
        WindEvent(timestamp=5.5, direction_deg=110, speed_kmph=18.0),
    )


def test_wind_events_non_monotonic_raises(make_plan):
    with pytest.raises(ValueError, match="must be strictly greater"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 5.0, direction_deg: 90, speed_kmph: 12.0}\n"
            "    - {timestamp: 3.0, direction_deg: 110, speed_kmph: 18.0}\n"
        )


def test_wind_events_equal_timestamps_raises(make_plan):
    with pytest.raises(ValueError, match="must be strictly greater"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 1.0, direction_deg: 90, speed_kmph: 12.0}\n"
            "    - {timestamp: 1.0, direction_deg: 110, speed_kmph: 18.0}\n"
        )


def test_wind_events_negative_timestamp_raises(make_plan):
    with pytest.raises(ValueError, match=r"timestamp=-1"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: -1.0, direction_deg: 90, speed_kmph: 12.0}\n"
        )


def test_wind_events_direction_out_of_range_raises(make_plan):
    with pytest.raises(ValueError, match=r"direction_deg=181"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 0.0, direction_deg: 181, speed_kmph: 12.0}\n"
        )


def test_wind_events_direction_180_ok(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_wind_sensor:\n"
        "    - {timestamp: 0.0, direction_deg: 180, speed_kmph: 12.0}\n"
    )
    assert plan.wind_events[0].direction_deg == 180


def test_wind_events_speed_over_ceiling_raises(make_plan):
    with pytest.raises(ValueError, match=r"speed_kmph=5000"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 0.0, direction_deg: 90, speed_kmph: 5000.0}\n"
        )


def test_wind_events_negative_speed_raises(make_plan):
    with pytest.raises(ValueError, match=r"speed_kmph=-1"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 0.0, direction_deg: 90, speed_kmph: -1.0}\n"
        )


def test_wind_events_missing_field_raises(make_plan):
    with pytest.raises(ValueError, match="missing required fields"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 0.0, direction_deg: 90}\n"
        )


def test_wind_events_unknown_field_raises(make_plan):
    with pytest.raises(ValueError, match="unknown fields"):
        make_plan(
            "events:\n"
            "  mock_wind_sensor:\n"
            "    - {timestamp: 0.0, direction_deg: 90, speed_kmph: 12.0, foo: bar}\n"
        )


def test_gps_events_parsed(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_gps:\n"
        "    - timestamp: 0.0\n"
        "      use_ocean_drift: true\n"
        "      ocean_drift_speed_kmph: 1.0\n"
        "      ocean_drift_dir_deg: 45.0\n"
        "    - {timestamp: 30.0, ocean_drift_speed_kmph: 2.5}\n"
    )
    assert plan.gps_events == (
        GpsEvent(
            timestamp=0.0,
            use_ocean_drift=True,
            ocean_drift_speed_kmph=1.0,
            ocean_drift_dir_deg=45.0,
        ),
        GpsEvent(timestamp=30.0, ocean_drift_speed_kmph=2.5),
    )


def test_gps_events_drift_speed_over_ceiling_raises(make_plan):
    with pytest.raises(ValueError, match="ocean_drift_speed_kmph"):
        make_plan(
            "events:\n"
            "  mock_gps:\n"
            "    - {timestamp: 0.0, ocean_drift_speed_kmph: 500.0}\n"
        )


def test_gps_events_drift_dir_out_of_range_raises(make_plan):
    with pytest.raises(ValueError, match="ocean_drift_dir_deg"):
        make_plan(
            "events:\n"
            "  mock_gps:\n"
            "    - {timestamp: 0.0, ocean_drift_dir_deg: 200.0}\n"
        )


def test_ais_events_snapshot_parsed(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_ais:\n"
        "    - timestamp: 0.0\n"
        "      ships:\n"
        "        - {id: 1, lat: 49.0, lon: -135.0, heading: 90.0, speed: 20.0,"
        " rot: 30, width: 1.0, height: 1.0}\n"
    )
    assert len(plan.ais_events) == 1
    event = plan.ais_events[0]
    assert event.timestamp == 0.0
    assert len(event.ships) == 1
    ship = event.ships[0]
    assert ship.id == 1
    assert ship.lat_lon.latitude == 49.0
    assert ship.cog.heading == 90.0


def test_ais_events_empty_ships_ok(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_ais:\n"
        "    - timestamp: 60.0\n"
        "      ships: []\n"
    )
    assert plan.ais_events == (AisEvent(timestamp=60.0, ships=()),)


def test_ais_events_ship_missing_field_raises(make_plan):
    with pytest.raises(ValueError, match="missing required fields"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 1, lat: 49.0, lon: -135.0, heading: 90.0, speed: 20.0,"
            " rot: 30, width: 1.0}\n"
        )


def test_ais_events_ship_invalid_heading_raises(make_plan):
    with pytest.raises(ValueError, match=r"heading=200"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 1, lat: 49.0, lon: -135.0, heading: 200.0, speed: 20.0,"
            " rot: 30, width: 1.0, height: 1.0}\n"
        )


def test_ais_events_ship_id_zero_raises(make_plan):
    with pytest.raises(ValueError, match=r"id=0 must be > 0"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 0, lat: 49.0, lon: -135.0, heading: 90.0, speed: 20.0,"
            " rot: 30, width: 1.0, height: 1.0}\n"
        )


def test_ais_events_duplicate_ship_ids_raises(make_plan):
    with pytest.raises(ValueError, match="duplicate ship id 1"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 1, lat: 49.0, lon: -135.0, heading: 90.0, speed: 20.0,"
            " rot: 30, width: 1.0, height: 1.0}\n"
            "        - {id: 1, lat: 48.0, lon: -134.0, heading: 45.0, speed: 15.0,"
            " rot: 0, width: 2.0, height: 2.0}\n"
        )


def test_ais_events_ship_speed_over_ceiling_raises(make_plan):
    with pytest.raises(ValueError, match=r"speed=1000"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 1, lat: 49.0, lon: -135.0, heading: 90.0, speed: 1000.0,"
            " rot: 30, width: 1.0, height: 1.0}\n"
        )


def test_ais_events_ship_lat_out_of_range_raises(make_plan):
    with pytest.raises(ValueError, match=r"lat=91"):
        make_plan(
            "events:\n"
            "  mock_ais:\n"
            "    - timestamp: 0.0\n"
            "      ships:\n"
            "        - {id: 1, lat: 91.0, lon: -135.0, heading: 90.0, speed: 20.0,"
            " rot: 30, width: 1.0, height: 1.0}\n"
        )


def test_global_path_events_parsed(make_plan):
    plan = make_plan(
        "events:\n"
        "  mock_global_path:\n"
        "    - timestamp: 0.0\n"
        "      waypoints:\n"
        "        - {latitude: 49.28, longitude: -123.18}\n"
        "        - {latitude: 48.16, longitude: -130.25}\n"
    )
    assert plan.global_path_events == (
        GlobalPathEvent(
            timestamp=0.0,
            waypoints=(
                ci.HelperLatLon(latitude=49.28, longitude=-123.18),
                ci.HelperLatLon(latitude=48.16, longitude=-130.25),
            ),
        ),
    )


def test_global_path_events_lat_out_of_range_raises(make_plan):
    with pytest.raises(ValueError, match=r"latitude=91"):
        make_plan(
            "events:\n"
            "  mock_global_path:\n"
            "    - timestamp: 0.0\n"
            "      waypoints:\n"
            "        - {latitude: 91.0, longitude: -123.0}\n"
            "        - {latitude: 48.16, longitude: -130.25}\n"
        )


def test_global_path_events_empty_waypoints_raises(make_plan):
    with pytest.raises(ValueError, match="waypoints"):
        make_plan(
            "events:\n"
            "  mock_global_path:\n"
            "    - timestamp: 0.0\n"
            "      waypoints: []\n"
        )


def test_global_path_events_single_waypoint_raises(make_plan):
    with pytest.raises(ValueError, match="at least 2 waypoints"):
        make_plan(
            "events:\n"
            "  mock_global_path:\n"
            "    - timestamp: 0.0\n"
            "      waypoints:\n"
            "        - {latitude: 49.28, longitude: -123.18}\n"
        )


def test_initial_global_path_single_waypoint_raises(make_plan):
    with pytest.raises(ValueError, match="at least 2 waypoints"):
        make_plan(
            "global_path:\n"
            "  waypoints:\n"
            "    - {latitude: 49.28, longitude: -123.18}\n"
        )


def test_unknown_mock_name_raises(make_plan):
    with pytest.raises(ValueError, match="Unknown mock names"):
        make_plan(
            "events:\n"
            "  mock_wind:\n"
            "    - {timestamp: 0.0, direction_deg: 90, speed_kmph: 12.0}\n"
        )
