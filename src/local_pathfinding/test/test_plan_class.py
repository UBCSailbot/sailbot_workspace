import io
from pathlib import Path

import pytest
from shapely.geometry import MultiPolygon
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci

BASIC_PLAN = "basic.yaml"


def load_test_plan(file_name: str) -> TestPlan:
    TestPlan._instance = None
    return TestPlan(file_name)


def test_instantiate_test_plan_object():
    plan = load_test_plan(BASIC_PLAN)
    assert isinstance(plan.land, MultiPolygon)
    assert len(plan.land.geoms) > 0
    assert len(plan.ais) > 0
    assert isinstance(plan.gps, ci.GPS)
    assert isinstance(plan.heading, ci.HelperHeading)
    assert -180.0 < plan.heading.heading <= 180.0
    assert isinstance(plan.global_path, ci.Path)
    assert isinstance(plan.tw_dir_deg, int)
    assert isinstance(plan.tw_speed_kmph, float)


def test_all_test_plans_use_separate_valid_heading() -> None:
    test_plan_dir = Path(__file__).resolve().parents[1] / "test_plans"
    for file_path in test_plan_dir.glob("*.yaml"):
        plan = load_test_plan(file_path.name)
        if plan.gps is None:
            assert plan.heading is None
        else:
            assert isinstance(plan.heading, ci.HelperHeading)
            assert -180.0 < plan.heading.heading <= 180.0
            assert plan.gps.heading.heading == 0.0


@pytest.mark.parametrize("heading_deg", [-180.0, 180.1, float("inf"), float("nan")])
def test_test_plan_rejects_invalid_heading(monkeypatch, heading_deg: float) -> None:
    yaml_text = (
        f"heading_deg: {heading_deg}\n"
        "gps:\n  latitude: 49.0\n  longitude: -123.0\n  speed_kmph: 5.0\n"
    )
    monkeypatch.setattr("builtins.open", lambda *_args, **_kwargs: io.StringIO(yaml_text))

    with pytest.raises(ValueError, match="heading_deg"):
        load_test_plan("invalid.yaml")


def test_test_plan_requires_heading_when_gps_is_present(monkeypatch) -> None:
    yaml_text = "gps:\n  latitude: 49.0\n  longitude: -123.0\n  speed_kmph: 5.0\n"
    monkeypatch.setattr("builtins.open", lambda *_args, **_kwargs: io.StringIO(yaml_text))

    with pytest.raises(ValueError, match="heading_deg is required"):
        load_test_plan("missing_heading.yaml")
