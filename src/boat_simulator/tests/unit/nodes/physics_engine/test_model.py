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
from tests.unit.nodes.physics_engine.test_kinematics_computation import ExpectedData


class TestBoatState:
    # pass
    def test_step_boat_state(
        self,
        glo_wind_vel,
        glo_water_vel,
        rudder_angle_deg,
        trim_tab_angle,
        input_data: KinematicsData,
        expected_output_data: ExpectedData,
        actual_input_data: KinematicsData,
    ) -> ExpectedData:
        # Assumes __compute_net_force_and_torque and __kinematics_computation.step work
        # __compute_net_force_and_torque needs to be tested

        # Set initial KinematicsData variables in pytest.mark.parameterize
        # Setting linear velocity gives the global velocity (for actual calculation)

        # Set global velocity manually for expected

        # net force and torque computed in BoatState.step, set manually in expected
        # compute all other intermediate vars manually w/ dif names

        relative_wind_vel = glo_wind_vel - input_data.linear_velocity
        relative_water_vel = glo_water_vel - input_data.linear_velocity

        actual_step = BoatState.step(glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle)

        return True
