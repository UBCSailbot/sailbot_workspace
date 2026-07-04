"""This module provides functionality for computing the lift and drag forces acting on a medium."""

import math
from typing import Tuple

import numpy as np
from rclpy.logging import get_logger

from boat_simulator.common.conventions import Body, Force, Velocity
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
        `fluid_density` (float): The density of the fluid acting on the medium, expressed in
            kilograms per cubic meter (kg/m^3).
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
            apparent_velocity (Vec2[Velocity, Body]): The apparent (relative) velocity between the
                fluid and the medium, expressed in meters per second (m/s).
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
            apparent_velocity (Vec2[Velocity, Body]): The apparent (relative) velocity between the
                fluid and the medium, calculated as the difference between the fluid velocity and
                the medium velocity (fluid_velocity - medium_velocity), expressed in meters per
                second (m/s).
            orientation (float): The orientation angle of the medium in degrees, where 0 degrees
                corresponds to the positive x-axis, and angles increase counter-clockwise (CCW).

        Returns:
            Tuple[float, float, float]: A tuple containing the lift force and drag
                force experienced by the medium, both expressed in newtons (N), and the angle of
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
                    the medium and the direction of the apparent velocity, expressed in degrees.

            Returns:
                Tuple[float, float, float]: A tuple of (lift_coefficient, drag_coefficient,
                    area). Both coefficients are unitless; area is in square meters (m^2).
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
    """Computes the hydrostatic restoring force. In the 4-DOF model this is nonzero only in
    roll, per the Hydrostatics design spec.

    Attributes:
        'seawater_density' (float) the density of seawater, expressed in kilograms per
            cubic meter (kg/m^3).
        'gravity' (float)) gravitational acceleration, expressed in meters per second squared
            (m/s^2).
        'displaced_volume' (float) the volume of water displaced by the boat at floating
            equilibrium, expressed in cubic meters (m^3).
        'metacentric_height' (float) TODO get the proper calculations for ts
        the vertical distance between the center of gravity
        and the metacenter, expressed in meters (m).
    """

    def __init__(
        self,
        seawater_density: float,
        gravity: float,
        displaced_volume: float,
        metacentric_height: float,
    ):
        self.__seawater_density = seawater_density
        self.__gravity = gravity
        self.__displaced_volume = displaced_volume
        self.__metacentric_height = metacentric_height

    def compute(self, roll_angle_rad: float) -> Vec4[Force, Body]:
        """Computes the hydrostatic restoring force at the given roll angle

        Args:
            roll_angle_rad (float): The boat's current roll angle, expressed in radians.

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

    Args:
        wing (MediumForceComputation): NACA 0018 lift/drag table and area for the main wing.
        tab (MediumForceComputation): NACA 0018 lift/drag table and area for the trim tab.
        chord_m (float): The main wing's mean chord length, expressed in meters (m).
        mast_pivot_chord_m (float): x_mast, the chordwise position of the mast pivot,
            expressed in meters (m) (typically the quarter-chord, per spec §2).
        boom_length_m (float): ell_tab, the distance from the mast axis to the trim tab's
            aerodynamic center, expressed in meters (m). TODO: TBD, mechanical CAD.
        wing_centre_of_effort (Tuple[float, float]): (x_s, z_s), the wing's center of effort
            relative to the boat's center of gravity, expressed in meters (m). TODO: TBD,
            mechanical CAD.
        air_density (float): rho_a, the density of air, expressed in kilograms per cubic meter
            (kg/m^3).
    """

    def __init__(
        self,
        wing: MediumForceComputation,
        tab: MediumForceComputation,
        chord_m: float,
        mast_pivot_chord_m: float,
        boom_length_m: float,
        wing_centre_of_effort: Tuple[float, float],
        air_density: float,
    ):
        self.__wing = wing
        self.__tab = tab
        self.__chord_m = chord_m
        self.__mast_pivot_chord_m = mast_pivot_chord_m
        self.__boom_length_m = boom_length_m
        self.__x_s, self.__z_s = wing_centre_of_effort
        self.__air_density = air_density

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
            float: The net pitching moment M_net(alpha), expressed in newton meters (N*m).
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
        x_s = lift_n * math.sin(theta) - drag_n * math.cos(theta)
        y_s = (lift_n * math.cos(theta) + drag_n * math.sin(theta)) * math.cos(roll_rad)

        f_y = lift_n * math.cos(alpha_rad) + drag_n * math.sin(alpha_rad)
        k_s = f_y * self.__z_s
        n_s = f_y * self.__x_s * math.cos(roll_rad)
        return Vec4.from_xypr(x_s, y_s, k_s, n_s)
