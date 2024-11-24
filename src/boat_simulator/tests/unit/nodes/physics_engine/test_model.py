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
    def test_step_no_motion(self):
        glo_wind_vel = np.zeros(3)
        glo_water_vel = np.zeros(3)
        rudder_angle_deg = 0
        trim_tab_angle = 0

        # Set initial KinematicsData variables
        # Setting linear velocity gives the global velocity (for actual calculation)
        # Default is 0 for all, don't need to set for this specific test

        # Set global velocity manually for expected

        # net force and torque computed in BoatState.step, set manually in expected
        # compute all other intermediate vars manually w/ dif names

        actual_step = BoatState.step(glo_wind_vel, glo_water_vel, rudder_angle_deg, trim_tab_angle)
