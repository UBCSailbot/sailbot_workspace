"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

from typing import Tuple

import numpy as np
import pytest
from numpy.typing import NDArray

from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.common.types import Scalar
from boat_simulator.nodes.physics_engine.kinematics_computation import BoatKinematics
from boat_simulator.nodes.physics_engine.kinematics_data import KinematicsData
from boat_simulator.nodes.physics_engine.model import BoatState


class TestBoatState:
    # pass
    def test_step_boat_state(
        self,
        timestep: Scalar,
        glo_wind_vel: NDArray,
        glo_water_vel: NDArray,
        rudder_angle_deg: Scalar,
        trim_tab_angle: Scalar,
        input_kin_data: NDArray,
    ):
        # Assumes __compute_net_force_and_torque and __kinematics_computation.step work
        # __compute_net_force_and_torque needs to be tested

        # Set initial KinematicsData variables in pytest.mark.parameterize
        # Setting linear velocity gives the global velocity (for actual calculation)

        # Set global velocity manually for expected

        # net force and torque computed in BoatState.step, set manually in expected
        # compute all other intermediate vars manually w/ dif names

        test_boat_state = BoatState(timestep)
        input_kinematics = KinematicsData(input_kin_data)

        relative_wind_vel = glo_wind_vel - input_kinematics.linear_velocity
        relative_water_vel = glo_water_vel - input_kinematics.linear_velocity

        total_force, total_torque = test_boat_state.__compute_net_force_and_torque(
            relative_wind_vel, relative_water_vel, rudder_angle_deg, trim_tab_angle
        )

        actual_step = test_boat_state.step(
            glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle
        )
        expected_step = test_boat_state.__kinematics_computation.step(total_force, total_torque)

        assert (actual_step == expected_step)

    @pytest.mark.parametrize(
        "timestep, glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle, input_kin_data",
        [
            (
                0.1,
                np.array([4.0, 5.5, 0.0], dtype=np.float32),
                np.array([1.0, 2.0, 0.0], dtype=np.float32),
                15,
                35,

            )
        ]
    )
