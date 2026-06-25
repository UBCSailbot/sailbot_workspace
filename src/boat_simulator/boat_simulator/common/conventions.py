"""Phantom-type markers for the simulator's typed vectors and matrices.

The classes in this module are *phantom types*: they are never instantiated and hold no
runtime data. They exist only to tag the generic containers in
:mod:`boat_simulator.common.types` (``Vec2`` / ``Vec3`` / ``Vec4`` and
``Mat3`` / ``Mat4``) with two
pieces of information:

* the **reference frame** a quantity is expressed in (:class:`NED` vs :class:`Body`), and
* the **physical quantity** it represents (:class:`Force`, :class:`Velocity`, ...).

A static type checker can then reject frame/quantity mismatches before they become silent
numerical bugs — for example passing a ``Vec3[Force, Body]`` where a
``Vec3[Velocity, NED]``
is expected.

TODO: We will need to finalize these types as we start integrating future calculations
We may want to combine Positions linear and angular values in a Vec4 setup like below,
η = [N, E, φ, ψ]ᵀ      # north (pose), east (pose), roll, yaw (heading)

Velocity can also be combined into Vec4 below,
ν = [u, v, p, r]ᵀ      # surge vel, sway vel, roll rate, yaw rate

Torque can also be combined into Vec4 as well below,
τ = [X, Y, K, N]ᵀ      # surge force, sway force, ROLL moment (3rd), YAW moment (4th)

For now, I will keep the type like below because combining units can cause confusion in our
computations
"""


# Reference-frame markers
class NED:
    """North-East-Down world frame: x north, y east, z down."""


class Body:
    """Boat body frame: x forward, y starboard (right), z down."""


# Quantity markers


class Position:
    """
    Position quantity, measured in metres (linear) or radians (angular).

    For Vec2,
    Linear Position: [x, y], metres
    Angular Position: [p, q] rad

    For Vec3,
    Linear Position: [x, y, z], metres
    Angular Position: [p, q, r] rad

    """


class Velocity:
    """
    Velocity quantity, measured in m/s (linear) or rad/s (angular).

    For Vec2,
    Linear Velocity: [vx, vy], metres
    Angular Velocity: [vp, vq] rad

    For Vec3,
    Linear Velocity: [vx, vy, vz], metres per second
    Angular Velocity: [vp, vq, vr], rad/s
    """


class Acceleration:
    """
    Acceleration quantity, measured in m/s² (linear) or rad/s² (angular).

    For Vec2,
    Linear Acceleration: [ax, ay], metres
    Angular Acceleration: [ap, aq] rad

    For Vec3,
    Linear Acceleration: [ax, ay, az], metres per second
    Angular Acceleration: [ap, aq, ar], rad/s
    """


class Force:
    """Force quantity, measured in newtons (N).

    Also used as the quantity tag for the generalized-force vector τ, whose components mix
    forces (N) and moments (N·m).

    For Vec2,
    Torque: [x, y], N·m

    For Vec3,
    Torque: [x, y, z], N·m
    """


class Torque:
    """
    Torque / moment quantity, measured in newton-metres (N·m).

    For Vec2,
    Torque: [x, y], N·m

    For Vec3,
    Torque: [x, y, z], N·m
    """


class LatLon:
    """Geographic latitude/longitude coordinate, measured in degrees."""


class Inertia:
    """Mass-inertia quantity of the boat, measured in kilogram-square-metres (kg·m²)."""


class InverseInertia:
    """
    Inverse Mass-inertia quantity of the boat, measured in (kilogram-square-metres)**-1
    (1/(kg·m²)).
    """
