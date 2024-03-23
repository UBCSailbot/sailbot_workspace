import math

from controller.common.lut import LUT


class WingsailController:
    """
    The controller class for computing trim tab angles for controlling the mainsail.

    Args:
    - chord_width_main_sail (float): The chord width of the main sail.
    - kinematic_viscosity (float): The kinematic viscosity of the fluid.
    - lut (LUT): A lookup table containing Reynolds numbers and corresponding desired angles of
      attack.
    """

    def __init__(self, chord_width_main_sail: float, kinematic_viscosity: float, lut: LUT):
        self.chord_width_main_sail = chord_width_main_sail
        self.kinematic_viscosity = kinematic_viscosity
        self.lut: LUT = lut

    def _compute_reynolds_number(self, apparent_wind_speed: float) -> float:
        """
        Computes the Reynolds number for the main sail.

        Args:
        - apparent_wind_speed (float): The apparent wind speed in meters per second.

        Returns:
        - reynolds_number (float): The computed Reynolds number for the main sail.
        """
        reynolds_number: float = (
            apparent_wind_speed * self.chord_width_main_sail
        ) / self.kinematic_viscosity
        return reynolds_number

    def _compute_trim_tab_angle(
        self, reynolds_number: float, apparent_wind_direction: float
    ) -> float:
        """
        Computes the trim tab angle based on Reynolds number and apparent wind direction. During
        this computation, it is assumed that the wingsail is aligned in the direction of the wind,
        with the rear of the wingsail (where the trim tab is located) pointing in the same
        direction as the wind is blowing.

        Args:
        - reynolds_number (float): The Reynolds number.
        - apparent_wind_direction (float): direction of the wind, in degrees. Here 0° means the
        apparent wind is blowing from the bow to the stern of the boat, the angle increases CCW.
        Range: -180 < direction <= 180 for symmetry

        Returns:
        - trim_tab_angle (float): The computed trim tab angle based on the provided
        Reynolds number and apparent wind direction.
        """
        desired_alpha: float = self.lut(reynolds_number)  # Using __call__ method
        # If the controller convention seems reversed, flip the sign of this return statement
        # EN, AE - 2024/03/16
        return math.copysign(desired_alpha, apparent_wind_direction)

    def get_trim_tab_angle(
        self, apparent_wind_speed: float, apparent_wind_direction: float
    ) -> float:
        """
        Computes and returns the final trim tab angle.

        Range: -40 <= direction <= 40 for symmetry. The realistic range of angles it can take is
        dictated by the angles used in the look up table.

        Args:
        - apparent_wind_speed (float): The apparent wind speed in meters per second.
        - apparent_wind_direction (float): The apparent wind direction in degrees.

        Returns:
        - trim_tab_angle (float): The computed trim tab angle. With angle increasing CCW
        """
        reynolds_number: float = self._compute_reynolds_number(apparent_wind_speed)
        trim_tab_angle: float = self._compute_trim_tab_angle(
            reynolds_number, apparent_wind_direction
        )
        return trim_tab_angle
