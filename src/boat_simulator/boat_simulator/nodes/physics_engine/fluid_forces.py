"""This module provides functionality for computing the lift and drag forces acting on a medium."""

import math
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from rclpy.logging import get_logger

from boat_simulator.common.constants import (
    AIR_DENSITY,
    BOAT_PROPERTIES,
    DISPLACED_VOLUME,
    EARTH_GRAVITY,
    METACENTRIC_HEIGHT,
    WATER_DENSITY,
)
from boat_simulator.common.conventions import (
    Acceleration,
    Body,
    Force,
    Velocity,
)
from boat_simulator.common.types import CoeffTable, Vec2, Vec4

_logger = get_logger(__name__)


class MediumForceComputation:
    """This class calculates the lift and drag forces experienced by a medium when subjected to
    fluid flow.

    Attributes:
        `lift_coefficients` (CoeffTable): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding lift
            coefficient.
        `drag_coefficients` (CoeffTable): An array of shape (n, 2) where each row contains a pair
            (x, y) representing an angle of attack, in degrees, and its corresponding drag
            coefficient.
        `areas` (float): A 2D area of the aero/hydrofoil.
        `fluid_density` (float): The density of the fluid acting on the medium, in
            kilograms per cubic meter.
    """

    def __init__(
        self,
        lift_coefficients: CoeffTable,
        drag_coefficients: CoeffTable,
        areas: float,
        fluid_density: float,
    ):
        self.__lift_coefficients = lift_coefficients
        self.__drag_coefficients = drag_coefficients
        self.__areas = areas
        self.__fluid_density = fluid_density

    def calculate_attack_angle(
        self, apparent_velocity: Vec2[Velocity, Body], orientation: float
    ) -> float:
        """Calculates the angle of attack formed between the orientation angle of the medium
        and the direction of the apparent velocity, bounded between -180 and 180 degrees.

        Args:
            apparent_velocity (Vec2[Velocity, Body]): The apparent relative velocity between the
                fluid and the medium, in meters per second.
            orientation (float): The orientation angle of the medium in degrees.

        Returns:
            float: The angle of attack formed between the orientation angle of the medium and
                the direction of the apparent velocity, expressed in degrees
                and bounded between -180 and 180 degrees.
        """
        velocity = apparent_velocity.data
        # Check if the apparent velocity is [0, 0]
        if np.all(velocity == 0):
            # Directly return the normalized orientation as the angle of attack
            # Normalize orientation to be within [-180, 180)
            angle_of_attack = ((orientation + 180) % 360) - 180
            _logger.debug(
                f"calculate_attack_angle: zero velocity, returning orientation={orientation:.2f} "
                + f"as aoa={angle_of_attack:.2f}",
            )
            return angle_of_attack

        # Calculate the angle in degrees of the apparent velocity
        angle_of_attack_raw = np.rad2deg(np.arctan2(velocity[1], velocity[0]))

        # Adjust orientation to be in the range of [-180, 180)
        orientation = ((orientation + 180) % 360) - 180

        # Calculate the raw angle of attack by subtracting the orientation from the velocity angle
        angle_of_attack = angle_of_attack_raw - orientation

        # Normalize the angle of attack to [-180, 180) range
        angle_of_attack = ((angle_of_attack + 180) % 360) - 180

        _logger.debug(
            f"calculate_attack_angle: vel={velocity} orientation={orientation:.2f} "
            + f"raw={angle_of_attack_raw:.2f} aoa={angle_of_attack:.2f}"
        )
        return angle_of_attack

    def compute(
        self, apparent_velocity: Vec2[Velocity, Body], orientation_deg: float
    ) -> Tuple[float, float, float]:
        """Computes the lift and drag forces experienced by a medium immersed in a fluid as well as
            the angle of attack to get them.

        Args:
            apparent_velocity (Vec2[Velocity, Body]): The apparent relative velocity between the
                fluid and the medium, in meters per second.
            orientation (float): The orientation angle of the medium in degrees, where 0 degrees
                corresponds to the positive x-axis, and angles increase counterclockwise.

        Returns:
            Tuple[float, float, float]: A tuple containing the lift force and drag
                force experienced by the medium, in Newtons, and the angle of
                attack.
        """

        attack_angle_deg = self.calculate_attack_angle(apparent_velocity, orientation_deg)
        lift_coefficient, drag_coefficient, area = self.interpolate(attack_angle_deg)
        velocity = apparent_velocity.data
        velocity_magnitude = np.linalg.norm(velocity)

        # With no relative flow there is no lift or drag (force ∝ |v|²).
        # Returning early also avoids the division by velocity_magnitude below, which
        # would otherwise produce NaN/Inf forces that propagate irreversibly through the
        # kinematics and blow up the simulation.
        if velocity_magnitude == 0:
            _logger.info("compute: zero apparent velocity, returning zero lift/drag force")
            return 0.0, 0.0, attack_angle_deg

        # Calculate the lift and drag forces
        lift_n = 0.5 * self.__fluid_density * lift_coefficient * area * velocity_magnitude**2
        drag_n = 0.5 * self.__fluid_density * drag_coefficient * area * velocity_magnitude**2

        _logger.info(
            f"compute: apparent_vel={velocity} m/s orientation={orientation_deg:6.2f}° "
            f"aoa={attack_angle_deg:6.2f}° "
            f"C_l={lift_coefficient:6.3f} C_d={drag_coefficient:6.3f} "
            f"area={area:.3f} m² |v|={velocity_magnitude:6.2f} m/s "
            f"L={lift_n} N "
            f"D={drag_n} N"
        )
        return lift_n, drag_n, attack_angle_deg

    def interpolate(self, attack_angle: float) -> Tuple[float, float, float]:
        """Performs linear interpolation to estimate the lift coefficient, drag coefficient, and
        area upon which the fluid acts, based on the provided angle of attack.

            Args:
                attack_angle (float): The angle of attack formed between the orientation angle of
                    the medium and the direction of the apparent velocity, in degrees.

            Returns:
                Tuple[float, float, float]: A tuple of (lift_coefficient, drag_coefficient,
                    area). Both coefficients are unitless; area is in meters squared.
        """

        # The foils are symmetric, so the lookup tables only store the positive AoA branch:
        # C_l is odd (C_l(-a) = -C_l(a)) and C_d is even (C_d(-a) = C_d(a)). We interpolate on
        # |AoA| and re-apply the sign to lift. Beyond the table's max angle the foil is fully
        # stalled and outside the modeled regime, so we return zero (np.interp would otherwise
        # silently clamp to the endpoint, producing peak lift at all out-of-range angles →
        # runaway thrust). Tables are assumed to start at 0°.
        abs_attack_angle = abs(attack_angle)
        lift_sign = float(np.sign(attack_angle))

        if abs_attack_angle > self.__lift_coefficients.max_angle:
            lift_coefficient = 0.0
        else:
            lift_coefficient = lift_sign * self.__lift_coefficients.interpolate(abs_attack_angle)

        if abs_attack_angle > self.__drag_coefficients.max_angle:
            drag_coefficient = 0.0
        else:
            drag_coefficient = self.__drag_coefficients.interpolate(abs_attack_angle)

        # The foil area is modeled as constant (independent of angle of attack).
        area = self.__areas

        return lift_coefficient, drag_coefficient, area

    @property
    def lift_coefficients(self) -> CoeffTable:
        return self.__lift_coefficients

    @property
    def drag_coefficients(self) -> CoeffTable:
        return self.__drag_coefficients

    @property
    def areas(self) -> float:
        return self.__areas

    @property
    def fluid_density(self) -> float:
        return self.__fluid_density


class HydroStaticsForceComputation:
    """Computes the hydrostatic restoring force.

    Attributes:
        'seawater_density' (float) the density of seawater, in kilograms per
            cubic meter.
        'gravity' (float)) gravitational acceleration, in meters per second squared.
        'displaced_volume' (float) the volume of water displaced by the boat at floating
            equilibrium, in cubic meters.
        'metacentric_height' (float) TODO get the proper calculations for ts
        the vertical distance between the center of gravity
        and the metacenter, in meters.
    """

    def __init__(self):
        self.__seawater_density = WATER_DENSITY
        self.__gravity = EARTH_GRAVITY
        self.__displaced_volume = DISPLACED_VOLUME
        self.__metacentric_height = METACENTRIC_HEIGHT

    def compute(self, roll_angle_rad: float) -> Vec4[Force, Body]:
        """Computes the hydrostatic restoring force at the given roll angle

        Args:
            roll_angle_rad (float): The boat's current roll angle, in radians.

        Returns:
            Vec4[Force, Body]: The restoring force
        """
        k_restore = (
            -self.__seawater_density
            * self.__gravity
            * self.__displaced_volume
            * self.__metacentric_height
            * math.sin(roll_angle_rad)
        )
        _logger.info(
            f"HydroStatics.compute: roll={roll_angle_rad:.4f} rad K_restore={k_restore:.2f} N·m"
        )
        return Vec4.from_xypr(0.0, 0.0, k_restore, 0.0)


class AeroDynamicsForceComputation:
    """Computes the wingsail's generalized force and moment contribution.

    Attributes:
        wing (MediumForceComputation): The main wing's lift and drag force computation.
        tab (MediumForceComputation): The trim tab's lift and drag force computation.
        chord_m (float): The main wing's mean chord length, expressed in meters.
        mast_pivot_chord_m (float): x_mast, the chordwise position of the mast pivot,
            in meters.
        boom_length_m (float): ell_tab, the distance from the mast axis to the trim tab's
            aerodynamic center, in meters.
        (x_s, z_s) (Tuple[float, float]): The wing's center of effort
            relative to the boat's center of gravity, in meters.
        air_density (float): rho_a, the density of air, in kilograms per cubic meter.
    """

    def __init__(self):
        self.__wing = MediumForceComputation(
            BOAT_PROPERTIES.sail_lift_coeffs,
            BOAT_PROPERTIES.sail_drag_coeffs,
            BOAT_PROPERTIES.sail_areas,
            AIR_DENSITY,
        )
        # TODO The trim tab needs its own coefficient tables and area in BOAT_PROPERTIES;
        # the main wing's values are stand-ins.
        self.__tab = MediumForceComputation(
            BOAT_PROPERTIES.sail_lift_coeffs,
            BOAT_PROPERTIES.sail_drag_coeffs,
            BOAT_PROPERTIES.sail_areas,
            AIR_DENSITY,
        )
        # TODO Placeholder: derive the mean chord from the real wingsail geometry.
        self.__chord_m = 1.5
        # TODO Placeholder: measure the mast pivot's chordwise position (~25% chord assumed).
        self.__mast_pivot_chord_m = 0.25 * self.__chord_m
        # TODO Placeholder: measure the distance from the mast axis to the tab's aero center.
        self.__boom_length_m = 1.5
        # TODO sail_dist is the sail CE-to-pivot distance, not CE-to-CG; z_s (CE height
        # relative to the CG) is a placeholder until we have real geometry.
        self.__x_s = BOAT_PROPERTIES.sail_dist
        self.__z_s = -3.0
        self.__air_density = AIR_DENSITY

    def apparent_wind(
        self,
        boat_velocity: Vec4[Velocity, Body],
        true_wind_speed: float,
        true_wind_bearing_rad: float,
        heading_rad: float,
    ) -> Tuple[float, float]:
        """Computes the apparent wind speed and angle-off-the-bow at the sail.

        Args:
            boat_velocity (Vec4[Velocity, Body]): The boat's current generalized velocity,
                 in meters per second.
            true_wind_speed (float): V_w, the true wind speed generated by the Fluid
                Simulation, in meters per second.
            true_wind_bearing_rad (float): beta_w, the true wind direction as an NED bearing,
                in radians.
            heading_rad (float): psi, the boat's current heading, in radians.

        Returns:
            Tuple[float, float]: A tuple of (V_aw, theta), the apparent wind speed in meters
                per second, and the apparent wind angle off the bow in radians.
        """
        u, v = boat_velocity.x, boat_velocity.y
        gamma_w = true_wind_bearing_rad - heading_rad - math.pi
        u_aw = u - true_wind_speed * math.cos(gamma_w)
        v_aw = v - true_wind_speed * math.sin(gamma_w)
        V_aw = math.sqrt(u_aw**2 + v_aw**2)
        theta = math.atan2(v_aw, u_aw)
        return V_aw, theta

    def net_pitching_moment(self, alpha_rad: float, v_aw: float, delta_tab_rad: float) -> float:
        """Sets up teh equation for the net pitching moment (or rather it gives the value).

        Args:
            alpha_rad (float): Wing angle of attack, in radians.
            v_aw (float): Apparent wind speed, in meters per second.
            delta_tab_rad (float): Trim tab deflection, in radians.

        Returns:
            float: The net pitching moment M_net(alpha), in newton meters.
                we want this to be 0
        """
        lift_n, drag_n, _ = self.__wing.compute(
            Vec2.from_xy(v_aw * math.cos(alpha_rad), v_aw * math.sin(alpha_rad)),
            0.0,  # double check this maths out properly
        )
        x_cop = self.__chord_m * (0.195 + 0.305 * abs(math.sin(alpha_rad)))
        n_perp = lift_n * math.cos(alpha_rad) + drag_n * math.sin(alpha_rad)
        m_wing = n_perp * (x_cop - self.__mast_pivot_chord_m)

        alpha_tab_rad = alpha_rad - delta_tab_rad
        lift_coefficient, _, tab_area = self.__tab.interpolate(math.degrees(alpha_tab_rad))
        l_tab = 0.5 * self.__air_density * v_aw**2 * tab_area * lift_coefficient
        m_tab = l_tab * self.__boom_length_m
        return m_wing + m_tab

    def solve_wing_angle(  # double check ts
        self, v_aw: float, delta_tab_rad: float, alpha_guess_rad: float = 0.0
    ) -> float:
        """Solves for the wing's equilibrium angle of attack.

        Args:
            v_aw (float): The apparent wind speed, in meters per second (m/s).
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle, in radians.

        Returns:
            float: The solved wing angle of attack alpha, in radians.
        """
        low, high = -math.pi / 2, math.pi / 2
        f_low = self.net_pitching_moment(low, v_aw, delta_tab_rad)
        f_high = self.net_pitching_moment(high, v_aw, delta_tab_rad)
        if f_low * f_high > 0:
            _logger.error(
                "solve_wing_angle: no sign change on [-90, 90] deg bracket, "
                "returning previous alpha unchanged"
            )
            return alpha_guess_rad

        for i in range(50):
            mid = 0.5 * (low + high)
            f_mid = self.net_pitching_moment(mid, v_aw, delta_tab_rad)
            if f_low * f_mid <= 0:
                high, f_high = mid, f_mid
            else:
                low, f_low = mid, f_mid
            if abs(high - low) < 1e-6:
                break
        return mid

    def compute(
        self,
        boat_velocity: Vec4[Velocity, Body],
        roll_rad: float,
        true_wind_speed: float,
        true_wind_bearing_rad: float,
        heading_rad: float,
        delta_tab_rad: float,
        alpha_guess_rad: float = 0.0,
    ) -> Vec4[Force, Body]:
        """Computes the wingsail's force.

        Args:
            boat_velocity (Vec4[Velocity, Body]): The boat's current generalized velocity.
            roll_rad (float): The boat's current roll angle in radians.
            true_wind_speed (float): The true wind speed in meters per second.
            true_wind_bearing_rad (float): The true wind direction,
                in radians.
            heading_rad (float): The boat's current heading, in radians.
            delta_tab_rad (float): The trim tab deflection, in radians.
            alpha_guess_rad (float): The previous timestep's solved wing angle,
                in radians.

        Returns:
            Vec4[Force, Body]: Surge force, sway force, roll moment, and yaw moment
            in newtons for the first two and newton meters for the last two.
        """
        v_aw, theta = self.apparent_wind(
            boat_velocity, true_wind_speed, true_wind_bearing_rad, heading_rad
        )
        alpha_rad = self.solve_wing_angle(v_aw, delta_tab_rad, alpha_guess_rad)

        lift_n, drag_n, attack_deg = self.__wing.compute(
            Vec2.from_xy(v_aw * math.cos(theta), v_aw * math.sin(theta)), math.degrees(alpha_rad)
        )

        # Force and Moment calculations
        x_s = lift_n * math.sin(theta) - drag_n * math.cos(theta)
        y_s = (lift_n * math.cos(theta) + drag_n * math.sin(theta)) * math.cos(roll_rad)

        f_y = lift_n * math.cos(alpha_rad) + drag_n * math.sin(alpha_rad)
        k_s = f_y * self.__z_s
        n_s = f_y * self.__x_s * math.cos(roll_rad)
        return Vec4.from_xypr(x_s, y_s, k_s, n_s)


class HydroDynamicsForceComputation:
    """Computes the hydrodynamic forces acting on the boat.

    Attributes:
        rudder (MediumForceComputation): The rudder's lift and drag force computation.
        keel (MediumForceComputation): The keel's lift and drag force computation.
        rudder_effort (Tuple[float, float]): (x_r, z_r), the rudder's center of effort relative to
            the boat's center of gravity, in meters.
        keel_effort (Tuple[float, float]): (x_k, z_k), the keel's center of effort relative to the
            boat's center of gravity, in meters.
        hull_effort (Tuple[float, float, float]): (x_h, y_h, z_h), the hull's center of effort
            relative to the boat's center of gravity, in meters.
        hull_r1 (float): The hull's quadratic drag coefficient.
        hull_r2 (float): The hull's linear drag coefficient.
        m_a (Mat4): The added mass matrix.
        d (Mat4): The linear damping matrix.
        water_density (float): The density of water, in kilograms per cubic meter.
    """

    def __init__(self):
        self.__rudder = MediumForceComputation(
            BOAT_PROPERTIES.rudder_lift_coeffs,
            BOAT_PROPERTIES.rudder_drag_coeffs,
            BOAT_PROPERTIES.rudder_areas,
            WATER_DENSITY,
        )
        # TODO The keel needs its own coefficient tables and area in BOAT_PROPERTIES;
        # the rudder's values are stand-ins.
        self.__keel = MediumForceComputation(
            BOAT_PROPERTIES.rudder_lift_coeffs,
            BOAT_PROPERTIES.rudder_drag_coeffs,
            BOAT_PROPERTIES.rudder_areas,
            WATER_DENSITY,
        )
        # TODO rudder_dist is the rudder CE-to-pivot distance, not CE-to-CG; the rudder
        # sits aft of the CG hence the negative sign. z_r (CE depth below the CG) is a
        # placeholder until we have real geometry.
        self.__x_r = -BOAT_PROPERTIES.rudder_dist
        self.__z_r = 0.5
        # TODO Placeholder: measure the keel's center of effort relative to the CG.
        self.__x_k = 0.0
        self.__z_k = 0.5
        # TODO Placeholder: measure the hull's center of effort relative to the CG.
        self.__x_h, self.__y_h, self.__z_h = (0.0, 0.0, 0.0)
        self.__hull_r1 = BOAT_PROPERTIES.hull_drag_factor
        # TODO Placeholder: add a linear hull drag coefficient to BOAT_PROPERTIES.
        self.__hull_r2 = 0.0
        self.__m_a = BOAT_PROPERTIES.M_A
        self.__d = BOAT_PROPERTIES.D

    def relative_velocity(
        self,
        nu: Vec4[Velocity, Body],
        current_speed: float,
        current_bearing_rad: float,
        heading_rad: float,
    ) -> Vec4[Velocity, Body]:
        """Computes v_r = nu - v_c, the boat's velocity relative to the water.

        Args:
            nu (Vec4[Velocity, Body]): the boat's generalized velocity [u, v, p, r].
            current_speed (float): V_c, ocean current speed, in meters per second.
            current_bearing_rad (float): beta_c, current direction in radians.
            heading_rad (float): psi, the boat's current heading in radians.

        Returns:
            Vec4[Velocity, Body]: v_r = [u_r, v_r, p, r], the relative water velocity.
        """
        gamma_c = heading_rad - current_bearing_rad - math.pi
        u_c = -current_speed * math.cos(gamma_c)
        v_c = current_speed * math.sin(gamma_c)
        return Vec4.from_xypr(nu.x - u_c, nu.y - v_c, nu.p, nu.r)

    def added_mass_coriolis_matrix(self, v_r: Vec4[Velocity, Body]) -> NDArray[np.float64]:
        """Builds C_A.

        Args:
            v_r (Vec4[Velocity, Body]): relative-to-water velocity [u, v, p, r].

        Returns:
            NDArray[np.float64]: the 4x4 C_A(v_r) matrix.
        """
        x_udot = self.__m_a.data[0, 0]
        y_vdot = self.__m_a.data[1, 1]
        k_pdot = self.__m_a.data[2, 2]
        u, v, p = v_r.x, v_r.y, v_r.p
        c_a = np.zeros((4, 4))
        c_a[0, 3] = y_vdot * v
        c_a[1, 3] = x_udot * u
        c_a[2, 2] = k_pdot * p
        c_a[3, 0] = -y_vdot * v
        c_a[3, 1] = x_udot * u
        c_a[3, 2] = y_vdot * v
        return c_a

    def hull_force(self, v_r: Vec4[Velocity, Body], roll_rad: float) -> Vec4[Force, Body]:
        """Calculates hull forces and moments.

        Args:
            v_r (Vec4[Velocity, Body]): relative water velocity.
            roll_rad (Float): roll angle in radians.

        Returns:
            Vec4[Force, Body]: hull forces and moments.
        """
        u, v, p, r = v_r.x, v_r.y, v_r.p, v_r.r
        u_h = -u + r * self.__y_h
        v_h = (-v - r * self.__x_h + p * self.__z_h) / math.cos(roll_rad)
        water_speed_rel_to_hull = math.sqrt(u_h**2 + v_h**2)
        alpha_h = math.atan2(v_h, u_h)
        h_d = self.__hull_r1 * water_speed_rel_to_hull**2 + self.__hull_r2

        # Force and Moment calculations
        x = h_d * math.cos(alpha_h)
        y = -h_d * math.sin(alpha_h) * math.cos(roll_rad)
        k = -y * self.__z_h
        n = y * self.__x_h
        return Vec4.from_xypr(x, y, k, n)

    def rudder_force(
        self, v_r: Vec4[Velocity, Body], roll_rad: float, delta_r_rad: float
    ) -> Vec4[Force, Body]:
        """Calculates rudder forces and moments.

        Args:
            v_r (Vec4[Velocity, Body]): relative water velocity.
            roll_rad (float): roll angle in radians.
            delta_r_rad (float): rudder deflection in radians.

        Returns:
            Vec4[Force, Body]: rudder forces and moments.
        """

        u, v, p, r = v_r.x, v_r.y, v_r.p, v_r.r
        u_r = -u
        v_r_new = -v - r * self.__x_r + p * self.__z_r
        water_speed_rel_to_rudder = math.sqrt(u_r**2 + v_r_new**2)
        beta_r = math.atan2(v_r_new, u_r)
        alpha_r = beta_r + delta_r_rad

        lift_n, drag_n, _ = self.__rudder.compute(
            Vec2.from_xy(
                water_speed_rel_to_rudder * math.cos(alpha_r),
                water_speed_rel_to_rudder * math.sin(alpha_r),
            ),
            0.0,
        )

        # Force and Moment calculations
        x = lift_n * math.sin(alpha_r) - drag_n * math.cos(alpha_r)
        y = (lift_n * math.cos(alpha_r) + drag_n * math.sin(alpha_r)) * math.cos(roll_rad)
        k = -(lift_n * math.cos(alpha_r) + drag_n * math.sin(alpha_r)) * self.__z_r
        n = (
            (lift_n * math.cos(alpha_r) + drag_n * math.sin(alpha_r))
            * self.__x_r
            * math.cos(roll_rad)
        )
        return Vec4.from_xypr(x, y, k, n)

    def keel_force(self, v_r: Vec4[Velocity, Body], roll_rad: float) -> Vec4[Force, Body]:
        """Calculates keel forces and moments.

        Args:
            v_r (Vec4[Velocity, Body]): relative water velocity.
            roll_rad (float): roll angle in radians.

        Returns:
            Vec4[Force, Body]: keel forces and moments.
        """
        u, v, p, r = v_r.x, v_r.y, v_r.p, v_r.r
        u_k = -u
        v_k = v - r * self.__x_k - p * self.__z_k
        water_speed_rel_to_keel = math.hypot(u_k, v_k)
        beta_k = math.atan2(u_k, v_k)
        alpha_k = -beta_k + math.pi

        lift_n, drag_n, _ = self.__keel.compute(
            Vec2.from_xy(
                water_speed_rel_to_keel * math.cos(alpha_k),
                water_speed_rel_to_keel * math.sin(alpha_k),
            ),
            0.0,
        )

        # Force and Moment calculations
        x = -lift_n * math.sin(alpha_k) + drag_n * math.cos(alpha_k)
        y = -(lift_n * math.cos(alpha_k) - drag_n * math.sin(alpha_k)) * math.cos(roll_rad)
        k = (lift_n * math.cos(alpha_k) + drag_n * math.sin(alpha_k)) * self.__z_k
        n = (
            -(lift_n * math.cos(alpha_k) + drag_n * math.sin(alpha_k))
            * self.__x_k
            * math.cos(roll_rad)
        )
        return Vec4.from_xypr(x, y, k, n)

    def compute(
        self,
        v_r: Vec4[Velocity, Body],
        v_dot_r: Vec4[Acceleration, Body],
        roll_rad: float,
        delta_r_rad: float,
    ) -> Vec4[Force, Body]:
        """Calculates the total hydrodynamic generalized force acting on the boat.

        Args:
            v_r (Vec4[Velocity, Body]): The boat's velocity relative to the water.
            v_dot_r (Vec4[Acceleration, Body]): The time derivative of the boat's velocity
                relative to the water.
            roll_rad (float): The boat's current roll angle in radians.
            delta_r_rad (float): The rudder deflection angle in radians.

        Returns:
            Vec4[Force, Body]: [X, Y, K, N] total hydrodynamic contribution.
        """
        tau_h = self.hull_force(v_r, roll_rad)
        tau_r = self.rudder_force(v_r, roll_rad, delta_r_rad)
        tau_k = self.keel_force(v_r, roll_rad)

        added_mass_term = self.__m_a.data @ v_dot_r.data
        c_a = self.added_mass_coriolis_matrix(v_r)
        coriolis_term = c_a @ v_r.data
        damping_term = self.__d.data @ v_r.data

        total = (
            tau_h.data + tau_r.data + tau_k.data - added_mass_term - coriolis_term - damping_term
        )
        return Vec4(total)
