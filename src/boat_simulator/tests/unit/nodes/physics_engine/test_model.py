"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

from typing import Tuple

import numpy as np
import pytest
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.model import BoatState


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
    # Assumes __compute_net_force_and_torque and __kinematics_computation.step work
    def test_step_boat_state(
        self,
        timestep: Scalar,
        glo_wind_vel: NDArray,
        glo_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
        input_kin_data: NDArray,
    ):
        test_boat_state = BoatState(timestep)
        print(dir(test_boat_state))
        input_kinematics = KinematicsData(input_kin_data)

        relative_wind_vel = glo_wind_vel - input_kinematics.linear_velocity
        relative_water_vel = glo_water_vel - input_kinematics.linear_velocity

        total_force, total_torque = test_boat_state._BoatState__compute_net_force_and_torque(
            relative_wind_vel, relative_water_vel, rudder_angle_deg, trim_tab_angle
        )

        actual_step = test_boat_state.step(
            glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle
        )
        expected_step = test_boat_state._BoatState__kinematics_computation.step(
            total_force, total_torque
        )

        assert actual_step == expected_step
