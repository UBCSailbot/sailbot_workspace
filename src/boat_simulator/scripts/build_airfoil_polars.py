#!/usr/bin/env python3
"""Generate ``boat_simulator/common/airfoil_polars.py`` from airfoiltools.com polars.

For each foil the simulator models, this script downloads the XFOIL polar CSVs for its NACA
section at a range of Reynolds numbers, resamples the pre-stall branch to whole-degree angles of
attack, and extends each polar out to 90 deg with the Viterna-Corrigan flat-plate post-stall
model (a sailboat's sail and control surfaces routinely operate far past stall -- e.g. a sail near
90 deg dead downwind -- so the raw XFOIL range of roughly +/-18 deg is not enough).  The result is
written as an importable Python module of :class:`CoeffGrid` objects, keeping the large numeric
tables out of ``constants.py`` while staying diffable and free of package-data plumbing.

This is a one-off developer tool: run it to (re)generate the committed data module.  It is not
imported at runtime.

Usage:
    python3 src/boat_simulator/scripts/build_airfoil_polars.py

Reynolds interpolation between the generated tables happens at runtime in :class:`CoeffGrid`.
"""

from __future__ import annotations

import math
import textwrap
import urllib.request
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

# Standard Reynolds numbers airfoiltools.com publishes XFOIL polars at.
REYNOLDS_NUMBERS: Tuple[int, ...] = (50_000, 100_000, 200_000, 500_000, 1_000_000)

# airfoiltools polar-key template: xf-<airfoil>-<Re> (Ncrit 9, Mach 0).
_CSV_URL = "http://airfoiltools.com/polar/csv?polar=xf-{airfoil}-{reynolds}"

# Whole-degree angle-of-attack samples appended past the XFOIL range via Viterna extrapolation.
_POST_STALL_ANGLES_DEG: Tuple[float, ...] = (20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 75.0, 90.0)


@dataclass(frozen=True)
class Foil:
    """A modeled foil: which sim tables it feeds, its NACA section, and its aspect ratio.

    Attributes:
        name: Prefix used for the generated module constants (e.g. ``"sail"`` -> ``SAIL_*``).
        airfoil: airfoiltools airfoil id (e.g. ``"n0012-il"``).
        aspect_ratio: Planform aspect ratio, used only to set the post-stall drag plateau
            ``CD_max = 1.11 + 0.018 * AR`` (Viterna-Corrigan).  Approximate -- refine with the real
            geometry.
    """

    name: str
    airfoil: str
    aspect_ratio: float


# TODO: refine the aspect ratios from the real boat geometry; they only affect the post-stall drag
# plateau (CD_max).  Keel and rudder share the NACA 0012 polars but keep separate grids because
# their chords -- and hence operating Reynolds numbers -- differ.
FOILS: Tuple[Foil, ...] = (
    Foil(name="sail", airfoil="naca0018-il", aspect_ratio=5.0),
    Foil(name="tab", airfoil="naca0018-il", aspect_ratio=3.0),
    Foil(name="rudder", airfoil="n0012-il", aspect_ratio=3.0),
    Foil(name="keel", airfoil="n0012-il", aspect_ratio=3.0),
)


def fetch_polar(airfoil: str, reynolds: int) -> np.ndarray:
    """Download one airfoiltools polar CSV and return its ``(N, 3)`` [alpha, Cl, Cd] rows."""
    url = _CSV_URL.format(airfoil=airfoil, reynolds=reynolds)
    with urllib.request.urlopen(url, timeout=30) as response:  # noqa: S310 (trusted host)
        text = response.read().decode("utf-8")

    lines = text.splitlines()
    header_idx = next(i for i, line in enumerate(lines) if line.startswith("Alpha,"))
    rows: List[Tuple[float, float, float]] = []
    for line in lines[header_idx + 1 :]:
        parts = line.split(",")
        if len(parts) < 3 or not parts[0].strip():
            continue
        alpha, cl, cd = float(parts[0]), float(parts[1]), float(parts[2])
        rows.append((alpha, cl, cd))
    return np.array(rows, dtype=float)


def _viterna_coefficients(
    alpha_stall_rad: float, cl_stall: float, cd_stall: float, aspect_ratio: float
) -> Tuple[float, float, float, float]:
    """Return the Viterna-Corrigan constants (A1, A2, B1, B2) matched at the stall point."""
    cd_max = 1.11 + 0.018 * aspect_ratio
    b1 = cd_max
    a1 = b1 / 2.0
    sin_s, cos_s = math.sin(alpha_stall_rad), math.cos(alpha_stall_rad)
    b2 = (cd_stall - cd_max * sin_s**2) / cos_s
    a2 = (cl_stall - cd_max * sin_s * cos_s) * sin_s / cos_s**2
    return a1, a2, b1, b2


def build_tables(foil: Foil, polar: np.ndarray) -> Tuple[List[List[float]], List[List[float]]]:
    """Resample the positive-AoA branch and extend to 90 deg; return (lift_rows, drag_rows)."""
    positive = polar[polar[:, 0] >= 0.0]
    positive = positive[np.argsort(positive[:, 0])]
    alpha_fine, cl_fine, cd_fine = positive[:, 0], positive[:, 1], positive[:, 2]

    # Match Viterna at the last whole degree covered by XFOIL data, so the stored table is
    # continuous across the XFOIL -> Viterna seam.
    alpha_match_deg = float(math.floor(alpha_fine[-1]))
    pre_stall_angles = np.arange(0.0, alpha_match_deg + 1.0, 1.0)
    cl_pre = np.interp(pre_stall_angles, alpha_fine, cl_fine)
    cd_pre = np.interp(pre_stall_angles, alpha_fine, cd_fine)

    a1, a2, b1, b2 = _viterna_coefficients(
        math.radians(alpha_match_deg),
        float(np.interp(alpha_match_deg, alpha_fine, cl_fine)),
        float(np.interp(alpha_match_deg, alpha_fine, cd_fine)),
        foil.aspect_ratio,
    )

    lift_rows = [[float(a), round(float(c), 4)] for a, c in zip(pre_stall_angles, cl_pre)]
    drag_rows = [[float(a), round(float(c), 4)] for a, c in zip(pre_stall_angles, cd_pre)]
    for angle_deg in _POST_STALL_ANGLES_DEG:
        if angle_deg <= alpha_match_deg:
            continue
        rad = math.radians(angle_deg)
        cl = a1 * math.sin(2 * rad) + a2 * math.cos(rad) ** 2 / math.sin(rad)
        cd = b1 * math.sin(rad) ** 2 + b2 * math.cos(rad)
        lift_rows.append([angle_deg, round(cl, 4)])
        drag_rows.append([angle_deg, round(cd, 4)])
    return lift_rows, drag_rows


def _format_grid(name: str, tables_rows: List[List[List[float]]]) -> str:
    """Render a flake8-clean ``CoeffGrid`` literal for one foil coefficient (lift or drag)."""
    table_literals = []
    for rows in tables_rows:
        body = ",\n".join(f"                    [{a:.1f}, {v}]" for a, v in rows)
        table_literals.append(
            "        CoeffTable(\n"
            "            np.array(\n"
            "                [\n"
            + body
            + ",\n"
            "                ],\n"
            "                dtype=np.float64,\n"
            "            )\n"
            "        )"
        )
    joined = ",\n".join(table_literals)
    return f"{name} = CoeffGrid(\n    REYNOLDS_NUMBERS,\n    (\n{joined},\n    ),\n)"


def main() -> None:
    blocks: List[str] = []
    for foil in FOILS:
        lift_tables: List[List[List[float]]] = []
        drag_tables: List[List[List[float]]] = []
        for reynolds in REYNOLDS_NUMBERS:
            print(f"Fetching {foil.airfoil} Re={reynolds} for {foil.name}...")
            polar = fetch_polar(foil.airfoil, reynolds)
            lift_rows, drag_rows = build_tables(foil, polar)
            lift_tables.append(lift_rows)
            drag_tables.append(drag_rows)
        upper = foil.name.upper()
        naca = "".join(ch for ch in foil.airfoil.split("-")[0] if ch.isdigit())
        blocks.append(
            f"# {foil.name}: NACA {naca} "
            f"(aspect ratio {foil.aspect_ratio}), Viterna-extended to 90 deg.\n"
            + _format_grid(f"{upper}_LIFT_COEFFS", lift_tables)
            + "\n\n"
            + _format_grid(f"{upper}_DRAG_COEFFS", drag_tables)
        )

    header = textwrap.dedent(
        '''\
        """Reynolds-dependent lift/drag coefficient grids for the boat simulator's foils.

        GENERATED FILE -- do not edit by hand.  Regenerate with::

            python3 src/boat_simulator/scripts/build_airfoil_polars.py

        Each grid holds one XFOIL polar per Reynolds number (from airfoiltools.com), resampled to
        whole-degree angles of attack and extended past stall to 90 deg with the Viterna-Corrigan
        model.  Only the positive angle-of-attack branch is stored; the symmetric-foil sign
        convention (Cl odd, Cd even) is applied by the force computation at lookup time.
        """

        import numpy as np

        from boat_simulator.common.types import CoeffGrid, CoeffTable

        # Ascending Reynolds-number breakpoints shared by every grid below.
        REYNOLDS_NUMBERS = np.array(
        '''
    )
    reynolds_literal = (
        "    [" + ", ".join(f"{r}.0" for r in REYNOLDS_NUMBERS) + "],\n    dtype=np.float64,\n)"
    )

    output = header + reynolds_literal + "\n\n\n" + "\n\n\n".join(blocks) + "\n"

    out_path = (
        "/workspaces/sailbot_workspace/src/boat_simulator/boat_simulator/common/airfoil_polars.py"
    )
    with open(out_path, "w") as handle:
        handle.write(output)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
