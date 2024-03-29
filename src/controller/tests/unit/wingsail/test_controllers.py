import numpy as np
import pytest

from controller.common.constants import CHORD_WIDTH_MAIN_SAIL, KINEMATIC_VISCOSITY
from controller.common.lut import LUT
from controller.wingsail.controllers import WingsailController

# Define test data
test_lut_data = np.array(
    [[50000, 5.75], [100000, 6.75], [200000, 7], [500000, 9.25], [1000000, 10]]
)
test_lut = LUT(test_lut_data)
test_chord_width = CHORD_WIDTH_MAIN_SAIL
test_kinematic_viscosity = KINEMATIC_VISCOSITY


class TestWingsailController:
    """
    Tests the functionality of the WingsailController class.
    """

    @pytest.fixture
    def wingsail_controller(self):
        """
        Fixture to create an instance of WingsailController for testing.
        """
        return WingsailController(test_chord_width, test_kinematic_viscosity, test_lut)

    @pytest.mark.parametrize(
        "apparent_wind_speed, expected_reynolds_number",
        [
            (0, 0),
            (3, 3 * CHORD_WIDTH_MAIN_SAIL / KINEMATIC_VISCOSITY),
            (5, 5 * CHORD_WIDTH_MAIN_SAIL / KINEMATIC_VISCOSITY),
            (10, 10 * CHORD_WIDTH_MAIN_SAIL / KINEMATIC_VISCOSITY),
            (20, 20 * CHORD_WIDTH_MAIN_SAIL / KINEMATIC_VISCOSITY),
        ],
    )
    def test_compute_reynolds_number(
        self, wingsail_controller, apparent_wind_speed, expected_reynolds_number
    ):
        """
        Tests the computation of Reynolds number.

        Args:
            wingsail_controller: Instance of WingsailController.
        """
        computed_reynolds_number = wingsail_controller._compute_reynolds_number(
            apparent_wind_speed
        )
        assert np.isclose(computed_reynolds_number, expected_reynolds_number)

    @pytest.mark.parametrize(
        "reynolds_number, apparent_wind_direction, expected_trim_tab_angle",
        [
            (1250, 45.0, 5.75),
            (15388, -90.0, -5.75),
            (210945, 170.0, 7.0820875),
            (824000, -120.0, -9.736),
            (2000000, 0, 10.0),
        ],
    )
    def test_compute_trim_tab_angle(
        self,
        wingsail_controller,
        reynolds_number,
        apparent_wind_direction,
        expected_trim_tab_angle,
    ):
        """
        Tests the computation of trim tab angle.

        Args:
            wingsail_controller: Instance of WingsailController.
        """
        computed_trim_tab_angle = wingsail_controller._compute_trim_tab_angle(
            reynolds_number, apparent_wind_direction
        )
        assert np.isclose(computed_trim_tab_angle, expected_trim_tab_angle)

    @pytest.mark.parametrize(
        "apparent_wind_speed, apparent_wind_direction, expected_trim_tab_angle",
        [
            (10.0, 0, 6.9047300626451),
            (4.0, 90.0, 6.045136200464),
            (10.0, 180.0, 6.904730062645),
            (15.0, -50.0, -7.3212852819032),
            (20.0, -120.0, -7.928380375871),
        ],
    )
    def test_get_trim_tab_angle(
        self,
        wingsail_controller,
        apparent_wind_speed,
        apparent_wind_direction,
        expected_trim_tab_angle,
    ):
        """
        Tests the computation of final trim tab angle.

        Args:
            wingsail_controller: Instance of WingsailController.
        """
        computed_trim_tab_angle = wingsail_controller.get_trim_tab_angle(
            apparent_wind_speed, apparent_wind_direction
        )
        assert np.isclose(computed_trim_tab_angle, expected_trim_tab_angle)
