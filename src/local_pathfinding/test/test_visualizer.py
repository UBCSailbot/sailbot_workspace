import math
import pytest

import local_pathfinding.visualizer as viz
import local_pathfinding.coord_systems as cs


@pytest.mark.parametrize(
    "vec, expected, expect_unit",
    [
        # Normal vectors (should get normalized to length 1)
        (cs.XY(3.0, 4.0), cs.XY(0.6, 0.8), True),
        (cs.XY(-3.0, 4.0), cs.XY(-0.6, 0.8), True),
        (cs.XY(0.0, -5.0), cs.XY(0.0, -1.0), True),
        (cs.XY(1.0 / math.sqrt(2), 1.0 / math.sqrt(2)),
         cs.XY(1.0 / math.sqrt(2), 1.0 / math.sqrt(2)), True),

        # Zero or near-zero vectors (should return 0,0)
        (cs.XY(0.0, 0.0), cs.XY(0.0, 0.0), False),
        (cs.XY(1e-7, 0.0), cs.XY(0.0, 0.0), False),
        (cs.XY(0.0, -5e-7), cs.XY(0.0, 0.0), False),

        # at threshold, should give zero because mag > 1e-6
        (cs.XY(1e-6, 0.0), cs.XY(0.0, 0.0), False),

        # just above threshold → should normalize
        (cs.XY(1.1e-6, 0.0), cs.XY(1.0, 0.0), True),
    ]
)
def test_get_unit_vector(vec: cs.XY, expected: cs.XY, expect_unit: bool):
    result = viz.get_unit_vector(vec)

    # Compare components
    assert result == pytest.approx(expected), "incorrect unit vector output"

    # If expected to be a unit vector, magnitude must be ~1
    if expect_unit:
        mag = math.hypot(result.x, result.y)
        assert mag == pytest.approx(1.0), "magnitude of unit vector is not 1"
