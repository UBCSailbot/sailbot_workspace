"""Heel-dependent hydrostatic righting-arm calculations.

Replaces the constant-GM small-angle restoring moment K = -rho*g*V*GM*sin(phi) with a
righting-arm formulation K = -rho*g*V*GZ(phi), using (in order of fidelity):

  1. A GZ lookup table from heeled hydrostatics (FreeShip/DELFTship/Orca3D), a [N, 2]
     array of [heel_angle_deg, righting_arm_m] rows, odd-extended to negative heel.
  2. The wall-sided formula GZ = (GM + 0.5*BM*tan^2(phi))*sin(phi), valid until deck-edge
     immersion, held constant beyond it (conservative for a maneuvering sim).

Reference: Fossen (2011) Ch. 4 (restoring forces); any intact-stability text for the
wall-sided formula.
"""

import math
from typing import Optional

import numpy as np
from numpy.typing import NDArray


def compute_bm(waterplane_moment_of_inertia: float, displaced_volume: float) -> float:
    """BM = I_wp / V, the metacentric radius.

    Args:
        waterplane_moment_of_inertia (float): I_wp, the waterplane area's second moment
            about the longitudinal centerline axis (m^4). Measure from the waterplane
            sketch, e.g. via SolidWorks Section Properties.
        displaced_volume (float): V, the displaced volume at floating equilibrium (m^3).

    Returns:
        float: BM, in meters.
    """
    if displaced_volume <= 0.0:
        raise ValueError("displaced_volume must be positive")
    return waterplane_moment_of_inertia / displaced_volume


def compute_upright_gm(kb: float, bm: float, kg: float) -> float:
    """GM = KB + BM - KG, the upright metacentric height.

    Args:
        kb (float): Center of buoyancy height above the keel datum (m).
        bm (float): Metacentric radius (m), see `compute_bm`.
        kg (float): Center of gravity height above the keel datum (m).

    Returns:
        float: GM, in meters.
    """
    return kb + bm - kg


def righting_arm(
    heel_angle_rad: float,
    gm: float,
    bm: float,
    deck_edge_immersion_rad: float,
    gz_table: Optional[NDArray[np.float64]] = None,
) -> float:
    """Computes the righting arm GZ(phi), in meters. Odd in phi: GZ(-phi) = -GZ(phi).

    Args:
        heel_angle_rad (float): phi, the boat's current roll angle, in radians.
        gm (float): Upright metacentric height GM, in meters.
        bm (float): Metacentric radius BM, in meters.
        deck_edge_immersion_rad (float): Heel angle at which the deck edge immerses; the
            wall-sided formula is invalid past this point, so GZ is held constant beyond
            it. Ignored when `gz_table` is given.
        gz_table (Optional[NDArray]): A [N, 2] array of [heel_angle_rad, gz_m] rows for
            angles >= 0, overriding the wall-sided formula when present.

    Returns:
        float: GZ, in meters.
    """
    phi = float(heel_angle_rad)
    sign = math.copysign(1.0, phi)
    abs_phi = abs(phi)

    if gz_table is not None:
        # np.interp clamps to the table's last value beyond its covered range, which is
        # the conservative choice for a maneuvering sim (holding the righting arm rather
        # than losing it).
        return sign * float(np.interp(abs_phi, gz_table[:, 0], gz_table[:, 1]))

    effective_phi = min(abs_phi, deck_edge_immersion_rad)
    tan_phi = math.tan(effective_phi)
    gz = (gm + 0.5 * bm * tan_phi * tan_phi) * math.sin(effective_phi)
    return sign * gz


def restoring_roll_moment(
    heel_angle_rad: float,
    gm: float,
    bm: float,
    deck_edge_immersion_rad: float,
    displaced_volume: float,
    water_density: float,
    gravity: float,
    gz_table: Optional[NDArray[np.float64]] = None,
) -> float:
    """Computes the restoring (righting) roll moment K_restoring, in N*m.

    The buoyancy couple opposes heel, hence the leading minus sign.

    Args:
        heel_angle_rad (float): phi, the boat's current roll angle, in radians.
        gm (float): Upright metacentric height, in meters.
        bm (float): Metacentric radius, in meters.
        deck_edge_immersion_rad (float): Heel angle at which the deck edge immerses.
        displaced_volume (float): V, the displaced volume at floating equilibrium (m^3).
        water_density (float): rho, the density of water (kg/m^3).
        gravity (float): g, gravitational acceleration (m/s^2).
        gz_table (Optional[NDArray]): See `righting_arm`.

    Returns:
        float: K_restoring, in newton meters.
    """
    gz = righting_arm(heel_angle_rad, gm, bm, deck_edge_immersion_rad, gz_table)
    return -water_density * gravity * displaced_volume * gz


def restoring_potential_energy(
    heel_angle_rad: float,
    gm: float,
    bm: float,
    deck_edge_immersion_rad: float,
    gz_table: Optional[NDArray[np.float64]] = None,
    num_integration_steps: int = 200,
) -> float:
    """Computes U(phi)/(rho*g*V) = integral of GZ(t) dt from 0 to |phi|, the potential
    energy stored against the righting moment, per unit of rho*g*V.

    Numerically integrated (trapezoidal) rather than solved in closed form so it stays
    correct for both the wall-sided formula and an arbitrary `gz_table`, and stays in
    sync automatically if `righting_arm`'s formula ever changes.

    Args:
        heel_angle_rad (float): phi, the boat's current roll angle, in radians.
        gm (float): Upright metacentric height, in meters.
        bm (float): Metacentric radius, in meters.
        deck_edge_immersion_rad (float): Heel angle at which the deck edge immerses.
        gz_table (Optional[NDArray]): See `righting_arm`.
        num_integration_steps (int): Number of trapezoidal steps used for the integral.

    Returns:
        float: The integral of GZ from 0 to |phi|, in meters (multiply by rho*g*V for
            joules).
    """
    abs_phi = abs(float(heel_angle_rad))
    if abs_phi == 0.0:
        return 0.0
    angles = np.linspace(0.0, abs_phi, num_integration_steps + 1)
    gz_values = np.array(
        [righting_arm(a, gm, bm, deck_edge_immersion_rad, gz_table) for a in angles]
    )
    return float(np.trapz(gz_values, angles))


def roll_stiffness_small_angle(
    gm: float, displaced_volume: float, water_density: float, gravity: float
) -> float:
    """Computes d(K_restoring)/d(phi) at phi = 0, in N*m/rad.

    Useful for the roll decay period T_roll = 2*pi*sqrt((I_xx + K_pdot) / roll_stiffness),
    a dock-test cross-check for GM.

    Args:
        gm (float): Upright metacentric height, in meters.
        displaced_volume (float): V, the displaced volume at floating equilibrium (m^3).
        water_density (float): rho, the density of water (kg/m^3).
        gravity (float): g, gravitational acceleration (m/s^2).

    Returns:
        float: The small-angle roll stiffness, in N*m/rad.
    """
    return water_density * gravity * displaced_volume * gm
