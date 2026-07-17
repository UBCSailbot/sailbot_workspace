import math
from collections import deque
from types import SimpleNamespace

import pytest

import custom_interfaces.msg as ci
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


def test_wind_box_is_below_main_plot():
    vs = SimpleNamespace(
        aw_vector_kmph=cs.XY(1.0, 0.0),
        tw_vector_kmph=cs.XY(0.0, 1.0),
        bw_vector_kmph=cs.XY(-1.0, 0.0),
    )
    wind_config = viz.configure_wind_box_elements(vs)
    fig = viz.initial_plot()
    viz.apply_layout(vs, fig, zoom_needed=False, last_range=None)

    wind_y_domain = wind_config.layout_config["yaxis2"]["domain"]
    assert wind_y_domain[1] < fig.layout.yaxis.domain[0]
    assert wind_config.background_info["y0"] == wind_y_domain[0]
    assert wind_config.background_info["y1"] == wind_y_domain[1]


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
    # reference only affects hover lat/lon (customdata), not the plotted x/y asserted below
    reference = ci.HelperLatLon(latitude=49.0, longitude=-123.0)
    t = viz.build_intermediate_trace(local_x, local_y, reference).to_plotly_json()
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
        assert j["x"] == [boat_xy[0]] + list(local_x[1:])
        assert j["y"] == [boat_xy[1]] + list(local_y[1:])


def test_build_ompl_yaw_trace():
    trace = viz.build_ompl_yaw_trace(
        local_x_km=[0.0, 1.0, 2.0, 3.0],
        local_y_km=[3.0, 2.0, 1.0, 0.0],
        headings_deg=[0.0, 90.0, 180.0, -90.0],
    ).to_plotly_json()

    assert trace["name"] == "OMPL Yaw"
    assert trace["x"] == [0.0, 1.0, 2.0, 3.0]
    assert trace["y"] == [3.0, 2.0, 1.0, 0.0]
    assert trace["marker"]["symbol"] == "arrow"
    assert trace["marker"]["angleref"] == "up"
    # Plotly normalizes marker angles into [-180, 180].
    assert trace["marker"]["angle"] == pytest.approx([0.0, 90.0, -180.0, -90.0])
    assert [row[2] for row in trace["customdata"]] == pytest.approx(
        [math.pi / 2, 0.0, -math.pi / 2, -math.pi]
    )


def test_build_ompl_yaw_trace_rejects_mismatched_data():
    with pytest.raises(ValueError, match="matching lengths"):
        viz.build_ompl_yaw_trace([0.0], [0.0], [])


def make_visualizer_message() -> ci.LPathData:
    gps = ci.GPS(
        lat_lon=ci.HelperLatLon(latitude=49.0, longitude=-123.0),
        speed=ci.HelperSpeed(speed=5.0),
        heading=ci.HelperHeading(heading=123.0),
    )
    global_path = ci.Path(
        waypoints=[
            ci.HelperLatLon(latitude=49.1, longitude=-123.1),
            ci.HelperLatLon(latitude=49.0, longitude=-123.0),
        ]
    )
    local_path = ci.Path(
        waypoints=[
            ci.HelperLatLon(latitude=49.0, longitude=-123.0),
            ci.HelperLatLon(latitude=49.05, longitude=-123.05),
        ]
    )
    return ci.LPathData(
        global_path=global_path,
        local_path=local_path,
        gps=gps,
        filtered_wind_sensor=ci.WindSensor(
            speed=ci.HelperSpeed(speed=10.0), direction=0
        ),
        ais_ships=ci.AISShips(),
    )


def test_visualizer_uses_rudder_heading_for_boat_and_wind() -> None:
    rudder_heading = ci.HelperHeading(heading=90.0)
    state = viz.VisualizerState(
        msgs=deque([make_visualizer_message()]),
        heading=rudder_heading,
    )

    assert state.boat_heading_deg == 90.0
    assert state.boat_heading_deg != state.latest_msg.gps.heading.heading

    aw_direction_deg = viz.wcs.boat_to_global_coordinate(rudder_heading.heading, 0.0)
    expected_aw = viz.cs.polar_to_cartesian(math.radians(aw_direction_deg), 10.0)
    assert state.aw_vector_kmph == pytest.approx(expected_aw)

    trace = viz.build_boat_trace(state, boat_xy_km=(0.0, 0.0), dist_to_goal_km=1.0)
    assert trace.marker.angle == pytest.approx(
        viz.cs.true_bearing_to_plotly_cartesian(rudder_heading.heading)
    )
    assert "Heading: 90.0°" in trace.hovertemplate
