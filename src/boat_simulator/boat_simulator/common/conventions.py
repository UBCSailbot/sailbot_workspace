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
"""


# Reference-frame markers


class NED:
    """North-East-Down world frame: x north, y east, z down."""


class Body:
    """Boat body frame: x forward, y starboard (right), z down."""


# Quantity markers


class Position:
    """Position quantity, measured in metres (linear) or radians (angular)."""


class Velocity:
    """Velocity quantity, measured in m/s (linear) or rad/s (angular)."""


class Acceleration:
    """Acceleration quantity, measured in m/s² (linear) or rad/s² (angular)."""


class Force:
    """Force quantity, measured in newtons (N).

    Also used as the quantity tag for the generalized-force vector τ, whose components mix
    forces (N) and moments (N·m).
    """


class Torque:
    """Torque / moment quantity, measured in newton-metres (N·m)."""


class LatLon:
    """Geographic latitude/longitude coordinate, measured in degrees."""


class Inertia:
    """Mass-inertia quantity of the boat, measured in kilogram-square-metres (kg·m²)."""
