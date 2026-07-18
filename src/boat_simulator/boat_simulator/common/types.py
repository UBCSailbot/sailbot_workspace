"""Typed numerical containers used with convention phantom types.

The generic parameters on :class:`Vec2`, :class:`Vec3`, and :class:`Vec4` are
phantom types: they exist only for static type checking and have no runtime
storage cost.  For example, ``Vec3[Velocity, NED]`` cannot accidentally be
passed where ``Vec3[Force, Body]`` is expected when a type checker is used.

:class:`Mat3` and :class:`Mat4` carry the same ``[Quantity, Frame]`` phantom
types as the vectors, so a matrix represents a physical quantity expressed in a
frame — for example ``Mat3[Inertia, Body]`` for a body-frame inertia tensor.
Matrix ``@`` requires the operand to share the matrix's frame and leaves that
frame unchanged.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Generic, Sequence, TypeVar, Union, overload

import numpy as np
from numpy.typing import ArrayLike, NDArray

# Any attribute of a class that extends Enum
EnumAttr = TypeVar("EnumAttr", bound=Enum)

# Scalar value that can be an integer or a float.
Scalar = Union[int, float]

# Used in cases where support for scalars or arrays of scalars are needed.
ScalarOrArray = Union[Scalar, NDArray[Any]]

Quantity = TypeVar("Quantity")
Frame = TypeVar("Frame")
_Q = TypeVar("_Q")  # free Quantity for matrix @ vector / matrix overloads

# Internal validation helpers


def _validated_vector(data: ArrayLike, size: int) -> NDArray[np.float64]:
    """Return an immutable float copy of a vector after validating its shape."""
    array = np.asarray(data, dtype=float)
    if array.shape != (size,):
        raise ValueError(f"expected shape ({size},), got {array.shape}")

    # A frozen dataclass does not make a contained ndarray immutable.  Copy the
    # input so callers cannot mutate the vector through their original array,
    # then protect the stored array as well.
    array = array.copy()
    array.flags.writeable = False
    return array


def _validated_matrix(data: ArrayLike, rows: int, cols: int) -> NDArray[np.float64]:
    """Return an immutable float copy of a matrix after validating its shape."""
    array = np.asarray(data, dtype=float)
    if array.shape != (rows, cols):
        raise ValueError(f"expected shape ({rows}, {cols}), got {array.shape}")
    array = array.copy()
    array.flags.writeable = False
    return array


def _validated_coeff_table(data: ArrayLike) -> NDArray[np.float64]:
    """Return an immutable float copy of an (N, 2) lookup table after validating it.

    Column 0 is the angle of attack in degrees and must be sorted ascending (required for the
    linear interpolation in :meth:`CoeffTable.interpolate`); column 1 is the coefficient or area
    value.
    """
    array = np.asarray(data, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2:
        raise ValueError(f"expected shape (n, 2), got {array.shape}")
    if array.shape[0] < 1:
        raise ValueError("coefficient table must have at least one row")
    if np.any(np.diff(array[:, 0]) < 0):
        raise ValueError("angle-of-attack column (column 0) must be sorted in ascending order")
    array = array.copy()
    array.flags.writeable = False
    return array


# Vector types


@dataclass(frozen=True, eq=False, init=False)
class Vec3(Generic[Quantity, Frame]):
    """Immutable three-dimensional vector with quantity and frame type markers."""

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_vector(data, 3))

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Vec3[Quantity, Frame]:
        return cls(np.array([x, y, z], dtype=float))

    def __add__(self, other: Vec3[Quantity, Frame]) -> Vec3[Quantity, Frame]:
        if not isinstance(other, Vec3):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Vec3[Quantity, Frame]) -> Vec3[Quantity, Frame]:
        if not isinstance(other, Vec3):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Vec3[Quantity, Frame]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Vec3[Quantity, Frame]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Vec3) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Vec3, self.data.dtype.str, self.data.tobytes()))

    @property
    def x(self) -> float:
        return float(self.data[0])

    @property
    def y(self) -> float:
        return float(self.data[1])

    @property
    def z(self) -> float:
        return float(self.data[2])


@dataclass(frozen=True, eq=False, init=False)
class Vec2(Generic[Quantity, Frame]):
    """Immutable two-dimensional vector with quantity and frame type markers."""

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_vector(data, 2))

    @classmethod
    def from_xy(cls, x: float, y: float) -> Vec2[Quantity, Frame]:
        return cls(np.array([x, y], dtype=float))

    def __add__(self, other: Vec2[Quantity, Frame]) -> Vec2[Quantity, Frame]:
        if not isinstance(other, Vec2):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Vec2[Quantity, Frame]) -> Vec2[Quantity, Frame]:
        if not isinstance(other, Vec2):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Vec2[Quantity, Frame]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Vec2[Quantity, Frame]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Vec2) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Vec2, self.data.dtype.str, self.data.tobytes()))

    @property
    def x(self) -> float:
        return float(self.data[0])

    @property
    def y(self) -> float:
        return float(self.data[1])


@dataclass(frozen=True, eq=False, init=False)
class Vec4(Generic[Quantity, Frame]):
    """Immutable four-dimensional vector with quantity and frame type markers.

    Primary use is the three rigid-body state vectors in the 4-DOF formulation::

        η (eta):  Vec4[Position, NED]   — pose        [x, y, φ, ψ]
        ν (nu):   Vec4[Velocity, Body]  — velocity     [u, v, p, r]
        v̇ (nu_dot): Vec4[Acceleration, Body] — acceleration [u̇, v̇, ṗ, ṙ]
        τ (tau):  Vec4[Force,    Body]  — gen. force   [X, Y, K, N]

    The third component carries roll (φ) in η, roll-rate (r) in ν, and
    roll moment (N) in τ.
    The fourth component carries yaw (ψ) in η, yaw-rate (r) in ν, and
    yaw moment (N) in τ.
    """

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_vector(data, 4))

    @classmethod
    def from_xypr(cls, x: float, y: float, p: float, r: float) -> Vec4[Quantity, Frame]:
        return cls(np.array([x, y, p, r], dtype=float))

    def __add__(self, other: Vec4[Quantity, Frame]) -> Vec4[Quantity, Frame]:
        if not isinstance(other, Vec4):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Vec4[Quantity, Frame]) -> Vec4[Quantity, Frame]:
        if not isinstance(other, Vec4):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Vec4[Quantity, Frame]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Vec4[Quantity, Frame]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Vec4) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Vec4, self.data.dtype.str, self.data.tobytes()))

    @property
    def x(self) -> float:
        return float(self.data[0])

    @property
    def y(self) -> float:
        return float(self.data[1])

    @property
    def p(self) -> float:
        return float(self.data[2])

    @property
    def r(self) -> float:
        """Fourth component: yaw ψ in η, yaw-rate r in ν, yaw moment N in τ."""
        return float(self.data[3])


# Matrix type


@dataclass(frozen=True, eq=False, init=False)
class Mat3(Generic[Quantity, Frame]):
    """Immutable 3×3 matrix with quantity and frame type markers.

    Like :class:`Vec3`, the parameters are ``[Quantity, Frame]``: the matrix carries a physical
    quantity expressed in a reference frame, e.g. ``Mat3[Inertia, Body]`` for a body-frame inertia
    tensor.  ``@`` requires the operand to share the matrix's *Frame* and leaves that frame
    unchanged; the quantity tag is not transformed (the operand's quantity is passed through).
    """

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_matrix(data, 3, 3))

    # Factory methods

    @classmethod
    def identity(cls) -> Mat3[Quantity, Frame]:
        """Return the 3×3 identity matrix."""
        return cls(np.eye(3))

    # Arithmetic

    @overload
    def __matmul__(self, other: Vec3[_Q, Frame]) -> Vec3[_Q, Frame]: ...

    @overload
    def __matmul__(self, other: Mat3[_Q, Frame]) -> Mat3[_Q, Frame]: ...

    def __matmul__(self, other: object) -> object:
        if isinstance(other, Vec3):
            return Vec3(self.data @ other.data)
        if isinstance(other, Mat3):
            return Mat3(self.data @ other.data)
        return NotImplemented

    def __add__(self, other: Mat3[Quantity, Frame]) -> Mat3[Quantity, Frame]:
        if not isinstance(other, Mat3):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Mat3[Quantity, Frame]) -> Mat3[Quantity, Frame]:
        if not isinstance(other, Mat3):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Mat3[Quantity, Frame]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Mat3[Quantity, Frame]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Mat3) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Mat3, self.data.dtype.str, self.data.tobytes()))

    # Properties

    @property
    def T(self) -> Mat3[Quantity, Frame]:
        """Transpose — keeps the same quantity and frame tags."""
        return Mat3(self.data.T)


@dataclass(frozen=True, eq=False, init=False)
class Mat4(Generic[Quantity, Frame]):
    """Immutable 4×4 matrix with quantity and frame type markers.

    Like :class:`Vec4`, the parameters are ``[Quantity, Frame]``: the matrix carries a physical
    quantity expressed in a reference frame, e.g. ``Mat4[Inertia, Body]`` for a generalized
    body-frame mass-inertia matrix.  ``@`` requires the operand to share the matrix's *Frame* and
    leaves that frame unchanged; the quantity tag is not transformed (the operand's quantity is
    passed through).
    """

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_matrix(data, 4, 4))

    # Factory methods

    @classmethod
    def identity(cls) -> Mat4[Quantity, Frame]:
        """Return the 4×4 identity matrix."""
        return cls(np.eye(4))

    @classmethod
    def from_rotation_yaw(cls, yaw: float) -> Mat4[Quantity, Frame]:
        """Yaw rotation matrix for a yaw-only 4-DOF model.

        Maps a 4-vector ``[x, y, z, ψ]`` through a planar rotation about the z axis::

            ⎡ cos ψ  −sin ψ  0  0 ⎤
            ⎢ sin ψ   cos ψ  0  0 ⎥
            ⎢   0       0    1  0 ⎥
            ⎣   0       0    0  1 ⎦

        Args:
            yaw: Current heading ψ in radians (from :attr:`Heading.radians`).
        """
        c, s = math.cos(yaw), math.sin(yaw)
        return cls(
            np.array(
                [
                    [c, -s, 0.0, 0.0],
                    [s, c, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
        )

    # Arithmetic

    @overload
    def __matmul__(self, other: Vec4[_Q, Frame]) -> Vec4[_Q, Frame]: ...

    @overload
    def __matmul__(self, other: Mat4[_Q, Frame]) -> Mat4[_Q, Frame]: ...

    def __matmul__(self, other: object) -> object:
        if isinstance(other, Vec4):
            return Vec4(self.data @ other.data)
        if isinstance(other, Mat4):
            return Mat4(self.data @ other.data)
        return NotImplemented

    def __add__(self, other: Mat4[Quantity, Frame]) -> Mat4[Quantity, Frame]:
        if not isinstance(other, Mat4):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Mat4[Quantity, Frame]) -> Mat4[Quantity, Frame]:
        if not isinstance(other, Mat4):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Mat4[Quantity, Frame]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Mat4[Quantity, Frame]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Mat4) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Mat4, self.data.dtype.str, self.data.tobytes()))

    # Properties

    @property
    def T(self) -> Mat4[Quantity, Frame]:
        """Transpose — keeps the same quantity and frame tags."""
        return Mat4(self.data.T)


@dataclass(frozen=True, eq=False, init=False)
class CoeffTable:
    """Immutable angle-of-attack → coefficient lookup table.

    Stores an ``(N, 2)`` table whose column 0 is the angle of attack in degrees (sorted ascending)
    and column 1 is the corresponding lift/drag/moment coefficient or area value.  Used by
    :class:`~boat_simulator.nodes.physics_engine.fluid_forces.MediumForceComputation` to look up a
    coefficient for a given angle of attack via :meth:`interpolate`.
    """

    data: NDArray[np.float64]

    def __init__(self, data: ArrayLike) -> None:
        object.__setattr__(self, "data", _validated_coeff_table(data))

    @classmethod
    def from_rows(cls, rows: Sequence[Sequence[float]]) -> CoeffTable:
        """Build a table from a sequence of ``(angle_of_attack_deg, value)`` rows."""
        return cls(np.array(rows, dtype=float))

    @property
    def angles(self) -> NDArray[np.float64]:
        """The angle-of-attack column (degrees)."""
        return self.data[:, 0]

    @property
    def values(self) -> NDArray[np.float64]:
        """The coefficient / area column."""
        return self.data[:, 1]

    @property
    def min_angle(self) -> float:
        """Smallest tabulated angle of attack (degrees)."""
        return float(self.data[0, 0])

    @property
    def max_angle(self) -> float:
        """Largest tabulated angle of attack (degrees)."""
        return float(self.data[-1, 0])

    def interpolate(self, angle: float) -> float:
        """Linearly interpolate the tabulated value at ``angle`` (degrees).

        Angles outside ``[min_angle, max_angle]`` are clamped to the endpoint values, matching
        :func:`numpy.interp`.
        """
        return float(np.interp(angle, self.data[:, 0], self.data[:, 1]))

    def __len__(self) -> int:
        return int(self.data.shape[0])

    def __eq__(self, other: object) -> bool:
        return isinstance(other, CoeffTable) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((CoeffTable, self.data.dtype.str, self.data.tobytes()))
