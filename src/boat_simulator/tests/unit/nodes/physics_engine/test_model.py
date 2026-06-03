"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

import numpy as np
import pytest
from numpy.typing import NDArray

from custom_interfaces.msg import HelperLatLon

from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.model import BoatState

REFERENCE_LATLON = HelperLatLon(latitude=49.28, longitude=-123.185032)


class TestBoatState:
    @pytest.mark.parametrize(
        "timestep, glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle, input_kin_data",
        [
            # Generic test
            (
                0.1,
                np.array([-4.0, 5.5, 0.0], dtype=np.float32),
                np.array([1.0, -2.0, 0.0], dtype=np.float32),
                0,
                3,
                (
                    np.zeros(3, dtype=np.float32),
                    [2.0, 3.0, 0.0],
                    [1.5, 0.1, 0],
                    [0.2, 0.0, 3.0],
                    [0, 0.0, 1.1],
                    [-0.4, 0.0, 0],
                ),
            ),
            # Test for still water and no wind, angular velocity, or angular acceleration
            (
                0.005,
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                np.array([0.0, 0.0, 0.0], dtype=np.float32),
                15,
                0,
                (
                    [120.0, 52.7, 0.0],
                    [2.0, 3.0, 0.0],
                    [-1, 2, 0],
                    [0.2, 0.0, 0.08],
                    np.zeros(3, dtype=np.float32),
                    np.zeros(3, dtype=np.float32),
                ),
            ),
            # Test for long timestep, large parameters, negative angles
            (
                8,
                np.array([5.0, 9.2, 0.0], dtype=np.float32),
                np.array([-4.23, 3.0, 0.0], dtype=np.float32),
                -370,
                -2.6,
                (
                    [-2000, -4000, 0.0],
                    [6.0, 12.0, 0.0],
                    [3, -5, 0],
                    [0.0, 0.0, 2],
                    [0.3, 0.0, 3],
                    [-0.8, 0.0, 1.1],
                ),
            ),
        ],
    )
    # Tests BoatState.step()
    # __compute_net_force_and_torque and __kinematics_computation.step are tested elsewhere
    def test_step_boat_state(
        self,
        timestep: Scalar,
        glo_wind_vel: NDArray,
        glo_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
        input_kin_data: NDArray,
    ):
        test_boat_state = BoatState(timestep, REFERENCE_LATLON)
        input_kinematics = KinematicsData(input_kin_data)

        relative_wind_vel = glo_wind_vel[:2] - input_kinematics.linear_velocity[:2]
        relative_water_vel = glo_water_vel[:2] - input_kinematics.linear_velocity[:2]

        total_force, total_torque = getattr(
            test_boat_state,
            "_BoatState__compute_net_force_and_torque"
            )(
            relative_wind_vel, relative_water_vel, rudder_angle_deg, trim_tab_angle
        )

        actual_step = test_boat_state.step(
            glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle
        )
        expected_step = getattr(test_boat_state, "_BoatState__kinematics_computation").step(
            total_force, total_torque
        )

        assert actual_step == expected_step


class TestWingsailDynamics:
    """Tests the reduced-order wingsail rotational model in BoatState.__update_wing_angle."""

    # Apparent wind along +x, so wind_angle == 0 and angle-of-attack == -wing_angle.
    WIND = np.array([1.0, 0.0], dtype=np.float32)
    DT = 0.05

    @staticmethod
    def __update_fn(boat_state: BoatState):
        return getattr(boat_state, "_BoatState__update_wing_angle")

    def test_first_step_initializes_to_equilibrium(self):
        """The first step should snap to the tab-set equilibrium (no startup transient)."""
        boat_state = BoatState(self.DT, REFERENCE_LATLON)
        tab = 5.0
        wing_angle = self.__update_fn(boat_state)(self.WIND, tab)
        # theta_target = wind_angle - alpha_eq, with wind_angle == 0 and alpha_eq = gain * tab
        expected = -BOAT_PROPERTIES.trim_tab_gain * tab
        assert wing_angle == pytest.approx(expected, abs=1e-6)

    def test_rotates_gradually_toward_new_equilibrium(self):
        """After re-initializing at tab=0, a new tab command must be approached over time, not
        reached instantaneously (i.e. the wing has real dynamics)."""
        boat_state = BoatState(self.DT, REFERENCE_LATLON)
        update = self.__update_fn(boat_state)
        update(self.WIND, 0.0)  # initialize aligned with the wind (equilibrium for tab=0)

        tab = 5.0
        theta_target = -BOAT_PROPERTIES.trim_tab_gain * tab
        after_one_step = update(self.WIND, tab)
        assert abs(after_one_step - theta_target) > 1e-2  # not yet converged

    def test_converges_to_equilibrium_aoa(self):
        """Over many steps with a constant tab, the angle of attack settles at gain * tab."""
        boat_state = BoatState(self.DT, REFERENCE_LATLON)
        update = self.__update_fn(boat_state)
        update(self.WIND, 0.0)

        tab = 5.0
        for _ in range(2000):
            wing_angle = update(self.WIND, tab)

        aoa = 0.0 - wing_angle  # wind_angle is 0
        assert aoa == pytest.approx(BOAT_PROPERTIES.trim_tab_gain * tab, abs=0.1)

    def test_zero_tab_weathervanes_into_wind(self):
        """With no tab deflection the wing settles aligned with the wind (zero angle of attack)."""
        boat_state = BoatState(self.DT, REFERENCE_LATLON)
        update = self.__update_fn(boat_state)
        update(self.WIND, 5.0)  # initialize away from the wind

        for _ in range(2000):
            wing_angle = update(self.WIND, 0.0)

        assert wing_angle == pytest.approx(0.0, abs=0.1)  # aligned with wind_angle == 0

    def test_overshoot_is_bounded(self):
        """With damping ratio 0.7 the response should overshoot the target only slightly."""
        boat_state = BoatState(self.DT, REFERENCE_LATLON)
        update = self.__update_fn(boat_state)
        update(self.WIND, 0.0)

        tab = 5.0
        theta_target = -BOAT_PROPERTIES.trim_tab_gain * tab
        # Target is negative here, so the response undershoots; bound the excursion past it.
        min_angle = min(update(self.WIND, tab) for _ in range(2000))
        assert theta_target - min_angle < 0.15 * abs(theta_target)
