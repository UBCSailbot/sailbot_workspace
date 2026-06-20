"""Typed vectors, reference-frame markers, and angle value objects.

The generic parameters on :class:`Vec2`, :class:`Vec3`, and :class:`Vec4` are
phantom types: they exist only for static type checking and have no runtime
storage cost.  For example, ``Vec3[Velocity, NED]`` cannot accidentally be
passed where ``Vec3[Force, Body]`` is expected when a type checker is used.

:class:`Mat4` carries *two* phantom types — ``Mat4[FrameOut, FrameIn]`` — so
the type checker can verify that a matrix application transforms vectors from
the right source frame to the right target frame.

Rigid-body 4-DOF state vectors
-------------------------------
Following Fossen's notation for marine vehicles, three Vec4 roles appear
throughout the physics engine:

    η (eta) ``Vec4[Position, NED]``   — pose  [x, y, z, ψ]   in NED frame
    ν (nu)  ``Vec4[Velocity, Body]``  — velocity [u, v, w, r] in Body frame
    τ (tau) ``Vec4[Force, Body]``     — generalized force [X, Y, Z, N] in Body frame

The kinematic equation ``η̇ = J(ψ) ν`` uses ``J: Mat4[NED, Body]`` produced by
:meth:`Mat4.from_rotation_yaw`.  The dynamic equation ``M ν̇ = τ - …`` uses a
mass-inertia ``Mat4[Body, Body]``.

Physics-engine integration points
----------------------------------
* ``kinematics_data.KinematicsData``     — replace raw NDArray fields with
  ``Vec3[Position, NED]``, ``Vec3[Velocity, NED]``, etc.
* ``kinematics_computation.BoatKinematics.step()`` — accept
  ``Vec3[Force, NED]`` / ``Vec3[Torque, Body]`` instead of bare NDArray.
* ``model.BoatState.step()``             — accept ``RudderSteeringAngle`` and
  ``TrimTabAngle`` instead of raw Scalar; call ``.degrees`` when forwarding to
  ``fluid_forces.MediumForceComputation.compute()``.
* ``fluid_forces.MediumForceComputation.compute()`` — accept
  ``Vec2[Velocity, Body]`` for the apparent-velocity argument.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Generic, TypeVar, overload

import numpy as np
from numpy.typing import NDArray

Quantity = TypeVar("Quantity")
Frame = TypeVar("Frame")
FrameIn = TypeVar("FrameIn")
FrameOut = TypeVar("FrameOut")
_Q = TypeVar("_Q")  # free Quantity for Mat4 matmul overloads
_F = TypeVar("_F")  # free Frame   for Mat4 composition overloads


# Internal validation helpers


def _validated_vector(data: NDArray[np.floating[Any]], size: int) -> NDArray[np.float64]:
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


def _validated_matrix(
    data: NDArray[np.floating[Any]], rows: int, cols: int
) -> NDArray[np.float64]:
    """Return an immutable float copy of a matrix after validating its shape."""
    array = np.asarray(data, dtype=float)
    if array.shape != (rows, cols):
        raise ValueError(f"expected shape ({rows}, {cols}), got {array.shape}")
    array = array.copy()
    array.flags.writeable = False
    return array


# Vector types


@dataclass(frozen=True, eq=False)
class Vec3(Generic[Quantity, Frame]):
    """Immutable three-dimensional vector with quantity and frame type markers."""

    data: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "data", _validated_vector(self.data, 3))

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


@dataclass(frozen=True, eq=False)
class Vec2(Generic[Quantity, Frame]):
    """Immutable two-dimensional vector with quantity and frame type markers."""

    data: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "data", _validated_vector(self.data, 2))

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


@dataclass(frozen=True, eq=False)
class Vec4(Generic[Quantity, Frame]):
    """Immutable four-dimensional vector with quantity and frame type markers.

    Primary use is the three rigid-body state vectors in the 4-DOF formulation::

        η (eta):  Vec4[Position, NED]   — pose        [x, y, z, ψ]
        ν (nu):   Vec4[Velocity, Body]  — velocity     [u, v, w, r]
        τ (tau):  Vec4[Force,    Body]  — gen. force   [X, Y, Z, N]

    The fourth component ``w`` carries yaw (ψ) in η, yaw-rate (r) in ν, and
    yaw moment (N) in τ.
    """

    data: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "data", _validated_vector(self.data, 4))

    @classmethod
    def from_xyzw(cls, x: float, y: float, z: float, w: float) -> Vec4[Quantity, Frame]:
        return cls(np.array([x, y, z, w], dtype=float))

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
    def z(self) -> float:
        return float(self.data[2])

    @property
    def w(self) -> float:
        """Fourth component: yaw ψ in η, yaw-rate r in ν, yaw moment N in τ."""
        return float(self.data[3])


# Matrix type


@dataclass(frozen=True, eq=False)
class Mat4(Generic[FrameOut, FrameIn]):
    """Immutable 4×4 matrix mapping vectors from *FrameIn* to *FrameOut*.

    Example roles in the rigid-body equations:

    ``J(ψ) : Mat4[NED, Body]``
        Kinematic transformation — maps body-frame velocity ν to NED-frame
        rates η̇ via ``η̇ = J(ψ) ν``.  Build with
        :meth:`Mat4.from_rotation_yaw`.

    ``M : Mat4[Body, Body]``
        Generalized mass-inertia matrix — appears in the dynamic equation
        ``M ν̇ = τ − C(ν)ν − D(ν)ν``.

    The :attr:`T` property returns ``Mat4[FrameIn, FrameOut]``, which is used
    when ``J(ψ)ᵀ`` is needed to project NED forces into the body frame.
    """

    data: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "data", _validated_matrix(self.data, 4, 4))

    # Factory methods

    @classmethod
    def identity(cls) -> Mat4[FrameOut, FrameIn]:
        """Return the 4×4 identity matrix."""
        return cls(np.eye(4))

    @classmethod
    def from_rotation_yaw(cls, yaw: float) -> Mat4[NED, Body]:
        """Kinematic transformation J(ψ) for a yaw-only 4-DOF model.

        Maps body-frame velocity ``[u, v, w, r]`` to NED-frame rates
        ``[ẋ, ẏ, ż, ψ̇]``::

            ⎡ cos ψ  −sin ψ  0  0 ⎤
            ⎢ sin ψ   cos ψ  0  0 ⎥
            ⎢   0       0    1  0 ⎥
            ⎣   0       0    0  1 ⎦

        Args:
            yaw: Current heading ψ in radians (from :attr:`Heading.radians`).
        """
        c, s = math.cos(yaw), math.sin(yaw)
        return Mat4(  # type: ignore[return-value]
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
    def __matmul__(self, other: Vec4[_Q, FrameIn]) -> Vec4[_Q, FrameOut]: ...

    @overload
    def __matmul__(self, other: Mat4[FrameIn, _F]) -> Mat4[FrameOut, _F]: ...

    def __matmul__(self, other: object) -> object:
        if isinstance(other, Vec4):
            return Vec4(self.data @ other.data)
        if isinstance(other, Mat4):
            return Mat4(self.data @ other.data)
        return NotImplemented

    def __add__(self, other: Mat4[FrameOut, FrameIn]) -> Mat4[FrameOut, FrameIn]:
        if not isinstance(other, Mat4):
            return NotImplemented
        return type(self)(self.data + other.data)

    def __sub__(self, other: Mat4[FrameOut, FrameIn]) -> Mat4[FrameOut, FrameIn]:
        if not isinstance(other, Mat4):
            return NotImplemented
        return type(self)(self.data - other.data)

    def __mul__(self, scalar: float) -> Mat4[FrameOut, FrameIn]:
        if not np.isscalar(scalar):
            return NotImplemented
        return type(self)(self.data * scalar)

    def __rmul__(self, scalar: float) -> Mat4[FrameOut, FrameIn]:
        return self * scalar

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Mat4) and bool(np.array_equal(self.data, other.data))

    def __hash__(self) -> int:
        return hash((Mat4, self.data.dtype.str, self.data.tobytes()))

    # Properties

    @property
    def T(self) -> Mat4[FrameIn, FrameOut]:
        """Transpose — swaps the input and output frame tags."""
        return Mat4(self.data.T)  # type: ignore[return-value]


# Reference-frame markers (phantom types)


class NED:
    """North-East-Down frame: x north, y east, z down."""


class Body:
    """Boat body frame: x forward, y starboard/right, z down."""


# Quantity markers (phantom types)


class Position:
    """Position quantity, measured in metres and radians"""


class Velocity:
    """Velocity quantity, measured in metres per second and rad per second"""


class Acceleration:
    """Acceleration quantity, measured in metres per second squared and rad per second squared"""


class Force:
    """Force quantity, measured in newtons.

    Also used as the *Quantity* tag for the generalized-force vector τ,
    whose components include both forces (N) and moments (N·m).
    """


class Torque:
    """Torque / moment quantity, measured in newton-metres."""
