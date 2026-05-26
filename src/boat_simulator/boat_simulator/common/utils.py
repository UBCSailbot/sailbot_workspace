"""Useful functions that could be used anywhere in the boat simulator package."""

import math
from typing import Union, overload

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar, ScalarOrArray


def rad_to_degrees(angle: Scalar) -> Scalar:
    """Converts an angle from radians to degrees.

    Args:
        `angle` (Scalar): Angle in radians.

    Returns:
        Scalar: Angle in degrees.
    """
    return angle * (180 / math.pi)


def degrees_to_rad(angle: Scalar) -> Scalar:
    """Converts an angle from degrees to radians.

    Args:
        `angle` (Scalar): Angle in degrees.

    Returns:
        Scalar: Angle in radians.
    """
    return angle * (math.pi / 180)


@overload
def bound_to_180(angle: Scalar, isDegrees: bool = True) -> Scalar: ...


@overload
def bound_to_180(
    angle: NDArray[Union[np.int32, np.float32]], isDegrees: bool = True
) -> NDArray[Union[np.int32, np.float32]]: ...


def bound_to_180(angle: ScalarOrArray, isDegrees: bool = True) -> ScalarOrArray:
    """Converts all angles to be within the range [-180, 180) degrees or [-π, π) radians.

    Args:
        `angles` (ScalarOrArray): Angle(s) to be bound.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        ScalarOrArray: Bounded angle(s). Output unit matches `isDegrees`.
    """
    bound = 180 if isDegrees else math.pi
    return angle - 2 * bound * ((angle + bound) // (2 * bound))


def bound_to_360(angle: ScalarOrArray, isDegrees: bool = True) -> ScalarOrArray:
    """Converts an angle to be in the range [0, 360) degrees.

    Args:
        `angle` (ScalarOrArray): Angle to be bound.
        `isDegrees` (bool, optional): True if the input is in degrees, and false for radians.
            Defaults to True.

    Returns:
        ScalarOrArray: Bounded angle. Output units matches `isDegrees`.
    """
    bound = 360 if isDegrees else (2 * math.pi)
    bound_angle = angle % bound

    return bound_angle


def ccw_straight_to_cw_north_deg(angle_deg: Scalar) -> Scalar:
    """Convert a bearing from the simulator's internal convention
    (0° pointing 'straight'/north-aligned, increasing CCW) to the
    HelperHeading convention (0° north, increasing CW), bounded to (-180, 180].

    Why: the physics engine reports `true_bearing` in math-convention
    (CCW-positive), but ROS message contracts (HelperHeading, DesiredHeading)
    specify compass-convention (CW-positive, 0° north).
    """
    return bound_to_180(-angle_deg, isDegrees=True)


def euler_zyx_to_quaternion(
    roll_rad: Scalar, pitch_rad: Scalar, yaw_rad: Scalar
) -> NDArray:
    """Convert intrinsic Z-Y-X Euler angles (roll about x, pitch about y,
    yaw about z) to a unit quaternion [x, y, z, w].

    Why: SimWorldState.global_pose.orientation is specified as a quaternion
    in ENU convention, but the physics engine stores angular position as
    Euler angles in radians.
    """
    cr, sr = math.cos(roll_rad / 2.0), math.sin(roll_rad / 2.0)
    cp, sp = math.cos(pitch_rad / 2.0), math.sin(pitch_rad / 2.0)
    cy, sy = math.cos(yaw_rad / 2.0), math.sin(yaw_rad / 2.0)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return np.array([x, y, z, w])


def get_wind_speed(wind: NDArray) -> Scalar:
    """Calculates wind speed.

    Args:
        wind (NDArray): Wind velocity in each coordinate.

    Returns:
        Scalar: Magnitude of wind velocity. Output units match 'wind'.
    """
    wind_squared = np.square(wind)
    sum_of_squares = np.sum(wind_squared)
    return math.sqrt(sum_of_squares)


def get_wind_direction(wind: NDArray) -> Scalar:
    """Calculates wind direction in x-y plane.

    Args:
        wind (NDArray): Wind velocity in each coordinate.

    Returns:
        Scalar: Direction of wind, in degrees. Angle is in [-180, 180).
    """
    angle_deg = round(np.arctan2(wind[0], wind[1]) * 180 / np.pi)
    return bound_to_180(angle_deg, isDegrees=True)
