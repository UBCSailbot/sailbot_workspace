"""Diagnostic: hand-compute path costs for mock paths to verify objective behavior.

Run from workspace root after sourcing the install:
    source install/local_setup.bash
    python3 notebooks/local_pathfinding/cost_diagnostic.py

Compares paths with different #tacks and different lateral swing widths so we can
check whether our cost functions actually rank "few + tight" cheaper than "many + wide".
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "src", "local_pathfinding"))

import local_pathfinding.coord_systems as cs  # noqa: E402
import local_pathfinding.ompl_objectives as obj  # noqa: E402


def compute_path_cost(waypoints, start, goal, tw_dir_rad_gc, tw_speed_kmph):
    """Sum each objective's per-segment cost across the path."""
    segments = list(zip(waypoints[:-1], waypoints[1:]))
    n_segments = len(segments)

    wind_total = sum(
        obj.WindObjective.wind_direction_cost(s1, s2, tw_dir_rad_gc) for s1, s2 in segments
    )
    time_total = sum(
        obj.TimeObjective.time_cost(s1, s2, tw_dir_rad_gc, tw_speed_kmph)
        for s1, s2 in segments
    )
    deviation_total = sum(
        obj.DeviationObjective.deviation_cost(s1, s2, start, goal) for s1, s2 in segments
    )
    segment_count_total = n_segments * obj.SEGMENT_COST

    weighted = {
        "wind": wind_total * obj.WIND_OBJECTIVE_WEIGHT,
        "time": time_total * obj.TIME_OBJECTIVE_WEIGHT,
        "deviation": deviation_total * obj.DEVIATION_OBJECTIVE_WEIGHT,
        "segments": segment_count_total * obj.SEGMENT_COUNT_OBJECTIVE_WEIGHT,
    }
    return n_segments, weighted, sum(weighted.values())


def zigzag(n_tacks, total_distance_km, lateral_swing_km):
    """Build a zigzag path going from (total_distance_km, 0) to (0, 0).

    n_tacks segments, alternating peaks/troughs at +lateral_swing_km / 0.
    """
    waypoints = []
    dx = total_distance_km / n_tacks
    for i in range(n_tacks + 1):
        x = total_distance_km - i * dx
        y = lateral_swing_km if (i % 2 == 1) else 0.0
        waypoints.append(cs.XY(x, y))
    return waypoints


def main():
    start = cs.XY(10.0, 0.0)
    goal = cs.XY(0.0, 0.0)
    tw_dir_rad_gc = math.radians(270.0)
    tw_speed_kmph = 15.0

    L = 10.0  # total westward distance

    paths = {
        "A: 2 tacks, close-hauled (~5 km swing)": zigzag(2, L, 5.0),
        "B: 2 tacks, wide (~10 km swing)": zigzag(2, L, 10.0),
        "C: 6 tacks, close-hauled (~1.67 km swing)": zigzag(6, L, 1.67),
        "D: 6 tacks, wide (~3.3 km swing)": zigzag(6, L, 3.3),
        "E: 12 tacks, close-hauled (~0.83 km swing)": zigzag(12, L, 0.83),
        "F: 12 tacks, wide (~2 km swing)": zigzag(12, L, 2.0),
    }

    header = (
        f"{'Path':<46} {'#segs':<6} "
        f"{'wind':>9} {'time':>9} {'devn':>9} {'segs':>9} {'TOTAL':>10}"
    )
    print(header)
    print("-" * len(header))
    for name, waypoints in paths.items():
        n, w, total = compute_path_cost(
            waypoints, start, goal, tw_dir_rad_gc, tw_speed_kmph
        )
        print(
            f"{name:<46} {n:<6} "
            f"{w['wind']:>9.3f} {w['time']:>9.3f} "
            f"{w['deviation']:>9.3f} {w['segments']:>9.3f} "
            f"{total:>10.3f}"
        )
    print()
    print(
        "Weights: "
        f"wind={obj.WIND_OBJECTIVE_WEIGHT}, time={obj.TIME_OBJECTIVE_WEIGHT}, "
        f"deviation={obj.DEVIATION_OBJECTIVE_WEIGHT}, "
        f"segment_count={obj.SEGMENT_COUNT_OBJECTIVE_WEIGHT}"
    )
    print(
        "Deviation knobs: "
        f"MAX_DEVIATION_FRACTION={obj.MAX_DEVIATION_FRACTION}, "
        f"DEVIATION_COST_EXPONENT={obj.DEVIATION_COST_EXPONENT}"
    )
    print()
    print("Expected ordering if objectives work as intended:")
    print("  - Tight beats wide at same #tacks: C<D, E<F (deviation cost)")
    print("  - Few beats many at same shape:    A<C<E (segment count cost)")


if __name__ == "__main__":
    main()
