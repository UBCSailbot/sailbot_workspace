"""Tests for the CoeffGrid Reynolds-dependent lookup in boat_simulator/common/types.py."""

import math

import numpy as np
import pytest

from boat_simulator.common.types import CoeffGrid, CoeffTable


def _linear_table(slope: float) -> CoeffTable:
    """A table whose value equals ``slope * angle`` at angles 0 and 10 degrees."""
    return CoeffTable(np.array([[0.0, 0.0], [10.0, 10.0 * slope]], dtype=np.float64))


class TestCoeffGrid:
    def test_from_single_is_reynolds_independent(self):
        grid = CoeffGrid.from_single(_linear_table(1.0))
        # Same value regardless of Reynolds number.
        for reynolds in (1.0, 1e3, 1e6, 1e9):
            assert math.isclose(grid.interpolate(5.0, reynolds), 5.0)

    def test_exact_reynolds_hit_returns_that_polar(self):
        grid = CoeffGrid(
            np.array([100.0, 1000.0], dtype=np.float64),
            (_linear_table(1.0), _linear_table(2.0)),
        )
        assert math.isclose(grid.interpolate(5.0, 100.0), 5.0)  # slope 1 polar
        assert math.isclose(grid.interpolate(5.0, 1000.0), 10.0)  # slope 2 polar

    def test_interpolates_linearly_in_log_reynolds(self):
        # Values 0.0 and 2.0 at Re 100 and 10000; the geometric-mean Re (1000) is the log-midpoint.
        grid = CoeffGrid(
            np.array([100.0, 10000.0], dtype=np.float64),
            (_linear_table(0.0), _linear_table(2.0)),
        )
        midpoint = grid.interpolate(10.0, 1000.0)
        assert math.isclose(midpoint, 10.0)  # halfway between 0 and 20

    def test_clamps_outside_reynolds_range(self):
        grid = CoeffGrid(
            np.array([100.0, 1000.0], dtype=np.float64),
            (_linear_table(1.0), _linear_table(2.0)),
        )
        # Below the lowest / above the highest Reynolds clamp to the end polars.
        assert math.isclose(grid.interpolate(5.0, 1.0), 5.0)
        assert math.isclose(grid.interpolate(5.0, 1e9), 10.0)

    def test_non_positive_reynolds_uses_lowest(self):
        grid = CoeffGrid(
            np.array([100.0, 1000.0], dtype=np.float64),
            (_linear_table(1.0), _linear_table(2.0)),
        )
        assert math.isclose(grid.interpolate(5.0, 0.0), 5.0)
        assert math.isclose(grid.interpolate(5.0, -10.0), 5.0)

    def test_max_angle_is_minimum_over_tables(self):
        short = CoeffTable(np.array([[0.0, 0.0], [45.0, 1.0]], dtype=np.float64))
        tall = CoeffTable(np.array([[0.0, 0.0], [90.0, 1.0]], dtype=np.float64))
        grid = CoeffGrid(np.array([100.0, 1000.0], dtype=np.float64), (short, tall))
        assert grid.max_angle == 45.0

    @pytest.mark.parametrize(
        "reynolds, tables",
        [
            (np.array([100.0]), (_linear_table(1.0), _linear_table(2.0))),  # count mismatch
            (np.array([1000.0, 100.0]), (_linear_table(1.0), _linear_table(2.0))),  # not ascending
            (np.array([0.0, 100.0]), (_linear_table(1.0), _linear_table(2.0))),  # non-positive Re
            (np.array([[100.0]]), (_linear_table(1.0),)),  # not 1-D
        ],
    )
    def test_invalid_construction_raises(self, reynolds, tables):
        with pytest.raises(ValueError):
            CoeffGrid(reynolds, tables)

    def test_single_table_grid_matches_underlying_table(self):
        table = _linear_table(1.5)
        grid = CoeffGrid.from_single(table)
        for angle in (0.0, 3.0, 7.5, 10.0):
            assert math.isclose(grid.interpolate(angle, 500.0), table.interpolate(angle))
