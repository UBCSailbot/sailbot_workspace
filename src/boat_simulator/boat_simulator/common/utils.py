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
def bound_to_180(angle: Scalar, isDegrees: bool = True) -> Scalar:
    ...


@overload
def bound_to_180(
    angle: NDArray[Union[np.int32, np.float32]], isDegrees: bool = True
) -> NDArray[Union[np.int32, np.float32]]:
    ...


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
