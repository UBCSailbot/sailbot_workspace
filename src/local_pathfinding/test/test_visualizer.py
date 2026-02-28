import math

import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.visualizer as viz


# --------------------------------------
# Math Helper Functions Tests
# --------------------------------------
@pytest.mark.parametrize(
    "last_goal, goal_xy, expect_msg",
    [
        (None, (1.23456, 7.89012), False),  # message first render
        ((1.235, 2.346), (1.2350001, 2.3459999), False),  # same message after rounding
        ((1.0, 2.0), (1.12345, 2.98765), True),  # should change message
    ],
)
def test_compute_goal_change(last_goal, goal_xy, expect_msg):
    gc = viz.compute_goal_change(last_goal, goal_xy)

    # always returns rounded
    assert gc.new_goal_xy_rounded == (
        round(goal_xy[0], viz.GOAL_CHANGE_ROUND_DECIMALS),
        round(goal_xy[1], viz.GOAL_CHANGE_ROUND_DECIMALS),
    )

    if expect_msg:
        assert gc.message is not None
        assert "Local goal advanced to" in gc.message
    else:
        assert gc.message is None


@pytest.mark.parametrize(
    "vec, expected, expect_unit",
    [
        # Normal vectors (should get normalized to length 1)
        (cs.XY(3.0, 4.0), cs.XY(0.6, 0.8), True),
        (cs.XY(-3.0, 4.0), cs.XY(-0.6, 0.8), True),
        (cs.XY(0.0, -5.0), cs.XY(0.0, -1.0), True),
        (
            cs.XY(1.0 / math.sqrt(2), 1.0 / math.sqrt(2)),
            cs.XY(1.0 / math.sqrt(2), 1.0 / math.sqrt(2)),
            True,
        ),
        # Zero or near-zero vectors (should return 0,0)
        (cs.XY(0.0, 0.0), cs.XY(0.0, 0.0), False),
        (cs.XY(1e-7, 0.0), cs.XY(0.0, 0.0), False),
        (cs.XY(0.0, -5e-7), cs.XY(0.0, 0.0), False),
        # at threshold, should give zero because mag > 1e-6
        (cs.XY(1e-6, 0.0), cs.XY(0.0, 0.0), False),
        # just above threshold → should normalize
        (cs.XY(1.1e-6, 0.0), cs.XY(1.0, 0.0), True),
    ],
)
def test_get_unit_vector(vec: cs.XY, expected: cs.XY, expect_unit: bool):
    result = viz.get_unit_vector(vec)

    # Compare components
    assert result == pytest.approx(expected), "incorrect unit vector output"

    # If expected to be a unit vector, magnitude must be ~1
    if expect_unit:
        mag = math.hypot(result.x, result.y)
        assert mag == pytest.approx(1.0), "magnitude of unit vector is not 1"


# --------------------------------------
# Figure Builder Functions Tests
# --------------------------------------
@pytest.mark.parametrize(
    "local_x, local_y, expected_x, expected_y, expected_text",
    [
        ([0.0, 1.0], [0.0, 1.0], [], [], []),  # less than 3 points case returns empty
        ([0.0, 10.0, 20.0], [0.0, 1.0, 2.0], [10.0], [1.0], ["LW1"]),
        ([0.0, 10.0, 20.0, 30.0], [0.0, 1.0, 2.0, 3.0], [10.0, 20.0], [1.0, 2.0], ["LW1", "LW2"]),
    ],
)
def test_build_intermediate_trace(local_x, local_y, expected_x, expected_y, expected_text):
    t = viz.build_intermediate_trace(local_x, local_y).to_plotly_json()
    assert t["x"] == expected_x
    assert t["y"] == expected_y

    # for empty case, plotly json might omit text; normalize it
    got_text = t.get("text", [])
    assert got_text == expected_text


@pytest.mark.parametrize(
    "local_x, local_y, boat_xy, expect_none",
    [
        ([], [], (0.0, 0.0), True),
        ([1.0], [], (0.0, 0.0), True),
        ([], [1.0], (0.0, 0.0), True),
        ([0.0, 1.0], [2.0, 3.0], (10.0, 20.0), False),
    ],
)
def test_build_path_trace(local_x, local_y, boat_xy, expect_none):
    t = viz.build_path_trace(local_x, local_y, boat_xy)
    if expect_none:
        assert t is None
    else:
        j = t.to_plotly_json()
        assert j["mode"] == "lines"
        assert j["name"] == "Path to Goal"
        assert j["x"] == list(local_x)
        assert j["y"] == list(local_y)
