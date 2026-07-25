"""Useful functions that could be used anywhere in the boat simulator package."""

import math
from typing import overload

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.conventions import Velocity
from boat_simulator.common.types import Frame, ScalarOrArray, Vec2


def rad_to_degrees(angle: float) -> float:
    """Converts an angle from radians to degrees.

    Args:
        `angle` (float): Angle in radians.

    Returns:
        float: Angle in degrees.
    """
    return angle * (180 / math.pi)


def degrees_to_rad(angle: float) -> float:
    """Converts an angle from degrees to radians.

    Args:
        `angle` (float): Angle in degrees.

    Returns:
        float: Angle in radians.
    """
    return angle * (math.pi / 180)


@overload
def bound_to_180(angle: float, isDegrees: bool = True) -> float: ...


@overload
def bound_to_180(angle: NDArray[np.float32], isDegrees: bool = True) -> NDArray[np.float32]: ...


def bound_to_180(
    angle: float | NDArray[np.float32], isDegrees: bool = True
) -> float | NDArray[np.float32]:
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


def bound_to_360(angle: float | NDArray[np.float32], isDegrees: bool = True) -> ScalarOrArray:
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


def ccw_straight_to_cw_north_deg(angle_deg: float) -> float:
    """Convert a bearing from the simulator's internal convention
    (0° pointing 'straight'/north-aligned, increasing CCW) to the
    HelperHeading convention (0° north, increasing CW), bounded to (-180, 180].

    Why: the physics engine reports `true_bearing` in math-convention
    (CCW-positive), but ROS message contracts (HelperHeading, DesiredHeading)
    specify compass-convention (CW-positive, 0° north).
    """
    return bound_to_180(-angle_deg, isDegrees=True)


def euler_zyx_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> NDArray:
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


def body_to_ned_rotation_matrix(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> NDArray[np.float64]:
    r"""Return Fossen's rotation matrix from the body frame to NED.

    This is :math:`R_{zyx}(\phi, \theta, \psi)` from the marine-craft model. Angles use radians
    and the convention roll ``phi`` about body x, pitch ``theta`` about body y, and yaw ``psi``
    about body z.

    Reference: https://fossen.biz/html/marineCraftModel.html

    Args:
        roll_rad: Roll angle ``phi`` in radians.
        pitch_rad: Pitch angle ``theta`` in radians.
        yaw_rad: Yaw angle ``psi`` in radians.

    Returns:
        A 3x3 orthonormal matrix such that ``v_ned = R_body_to_ned @ v_body``.

    Raises:
        ValueError: If any Euler angle is NaN or infinite.
    """
    angles = (roll_rad, pitch_rad, yaw_rad)
    if not all(math.isfinite(angle) for angle in angles):
        raise ValueError(f"Euler angles must be finite, got {angles}")

    cos_roll, sin_roll = math.cos(roll_rad), math.sin(roll_rad)
    cos_pitch, sin_pitch = math.cos(pitch_rad), math.sin(pitch_rad)
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)

    return np.array(
        [
            [
                cos_yaw * cos_pitch,
                -sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll,
                sin_yaw * sin_roll + cos_yaw * sin_pitch * cos_roll,
            ],
            [
                sin_yaw * cos_pitch,
                cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll,
                -cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll,
            ],
            [-sin_pitch, cos_pitch * sin_roll, cos_pitch * cos_roll],
        ],
        dtype=np.float64,
    )


def ned_to_body_rotation_matrix(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> NDArray[np.float64]:
    r"""Return the inverse rotation from NED to the body frame.

    This is the transpose of Fossen's orthonormal body-to-NED
    :math:`R_{zyx}(\phi, \theta, \psi)` matrix.

    Args:
        roll_rad: Roll angle ``phi`` in radians.
        pitch_rad: Pitch angle ``theta`` in radians.
        yaw_rad: Yaw angle ``psi`` in radians.

    Returns:
        A 3x3 matrix such that ``v_body = R_ned_to_body @ v_ned``.

    Raises:
        ValueError: If any Euler angle is NaN or infinite.
    """
    return body_to_ned_rotation_matrix(roll_rad, pitch_rad, yaw_rad).T


def get_wind_speed(wind: Vec2[Velocity, Frame]) -> float:
    """Calculates wind speed.

    Args:
        wind: Wind velocity in each coordinate.

    Returns:
        float: Magnitude of wind velocity. Output units match 'wind'.
    """
    wind_squared = np.square(wind.data)
    sum_of_squares = np.sum(wind_squared)
    return math.sqrt(sum_of_squares)


def get_wind_direction(wind: Vec2[Velocity, Frame]) -> float:
    """Calculates wind direction in x-y plane.

    Args:
        wind: Wind velocity in each coordinate.

    Returns:
        float: Flow-toward direction of the wind vector, in degrees. The angle
            is in [-180, 180).
    """
    angle_deg = round(np.arctan2(wind.x, wind.y) * 180 / np.pi)
    return bound_to_180(angle_deg, isDegrees=True)
