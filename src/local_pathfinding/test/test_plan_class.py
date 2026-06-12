from shapely.geometry import MultiPolygon
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci

BASIC_PLAN = "basic.yaml"


def test_instantiate_test_plan_object():
    plan = TestPlan(BASIC_PLAN)
    assert isinstance(plan.land, MultiPolygon)
    assert len(plan.land.geoms) > 0
    assert len(plan.ais) > 0
    assert isinstance(plan.gps, ci.GPS)
    assert isinstance(plan.global_path, ci.Path)
    assert isinstance(plan.tw_dir_deg, int)
    assert isinstance(plan.tw_speed_kmph, float)
