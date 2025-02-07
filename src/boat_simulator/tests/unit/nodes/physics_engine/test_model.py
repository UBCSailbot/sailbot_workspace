"""Tests classes and functions in boat_simulator/nodes/physics_engine/model.py"""

import numpy as np
import pytest

from boat_simulator.common.constants import BOAT_PROPERTIES
from boat_simulator.nodes.physics_engine.model import BoatState


@pytest.mark.parametrize(
    "rel_wind_vel, rel_water_vel, rudder_angle_deg, trim_tab_angle, timestep",
    [
        (np.array([1, 2]), np.array([1, 2]), 45, 45, 1),
        (np.array([1, 2]), np.array([1, 2]), 45, 45, 1),
        (np.array([4, 5]), np.array([7, 8]), 30, 60, 2),
        (np.array([10, 20]), np.array([15, 25]), 90, 120, 1),
        (np.array([0, 1]), np.array([1, 1]), 0, 90, 3),
        (np.array([-1, -2]), np.array([-1, -2]), 180, 270, 2),
        (np.array([3.5, 4.5]), np.array([1.5, 2.5]), 15, 75, 4),
        (np.array([100, 200]), np.array([300, 200]), 0, 45, 2),
        # cannot use 0 vector, or will error
    ],
)
def test_compute_net_force_torque(
    rel_wind_vel,
    rel_water_vel,
    rudder_angle_deg,
    trim_tab_angle,
    timestep,
):

    current_state = BoatState(timestep)
    net_force = current_state._BoatState__compute_net_force_and_torque(
        rel_wind_vel, rel_water_vel, rudder_angle_deg, trim_tab_angle
    )

    app_wind_vel = np.subtract(rel_wind_vel, current_state.relative_velocity[0:2])
    app_water_vel = np.subtract(rel_water_vel, current_state.relative_velocity[0:2])

    wind_angle = np.arctan2(app_wind_vel[1], app_wind_vel[0])
    trim_tab_angle_rad = np.radians(trim_tab_angle)
    main_sail_angle = wind_angle - trim_tab_angle_rad
    rudder_angle_rad = np.radians(rudder_angle_deg)

    test_sail_force = current_state._BoatState__sail_force_computation.compute(
        app_wind_vel, trim_tab_angle
    )
    test_rudder_force = current_state._BoatState__rudder_force_computation.compute(
        app_water_vel, rudder_angle_deg
    )

    hull_drag_force = current_state.relative_velocity[0:2] * BOAT_PROPERTIES.hull_drag_factor

    total_drag_force = test_sail_force[1] + test_rudder_force[1] + hull_drag_force
    total_force = test_sail_force[0] + test_rudder_force[0] + total_drag_force

    sail_drag = np.linalg.norm(test_sail_force[1], ord=2)
    sail_lift = np.linalg.norm(test_sail_force[0], ord=2)

    sail_lift_constant = (
        BOAT_PROPERTIES.mast_position[1]
        - (BOAT_PROPERTIES.sail_dist * np.cos(main_sail_angle))
        - BOAT_PROPERTIES.centre_of_gravity[1]
    )

    sail_drag_constant = BOAT_PROPERTIES.sail_dist * np.sin(main_sail_angle)

    sail_torque = np.add(sail_drag * sail_drag_constant, sail_lift * sail_lift_constant)

    rudder_drag = np.linalg.norm(test_rudder_force[1], ord=2)
    rudder_lift = np.linalg.norm(test_rudder_force[0], ord=2)

    rudder_drag_constant = BOAT_PROPERTIES.rudder_dist * np.sin(rudder_angle_rad)

    rudder_lift_constant = (
        BOAT_PROPERTIES.rudder_dist * np.cos(rudder_angle_rad)
        + BOAT_PROPERTIES.centre_of_gravity[1]
    )

    rudder_torque = np.add(rudder_lift * rudder_lift_constant, rudder_drag * rudder_drag_constant)

    total_torque = np.add(sail_torque, rudder_torque)

    final_torque = np.array([0, 0, total_torque])

    assert np.allclose(total_force, net_force[0], 0.5)  # checking force
    assert np.allclose(final_torque, net_force[1], 0.5)  # checking torque
