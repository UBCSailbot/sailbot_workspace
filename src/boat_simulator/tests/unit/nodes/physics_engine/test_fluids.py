"""Tests classes and functions in boat_simulator/nodes/physics_engine/fluids.py"""

import numpy as np
import pytest

from boat_simulator.common.generators import ConstantGenerator, MVGaussianGenerator
from boat_simulator.nodes.physics_engine.fluid_generation import FluidGenerator


class TestFluidGenerator:
    @pytest.mark.parametrize(
        "vector",
        [
            (np.array([1, 0, 1])),
            (np.array([0, 1, 0])),
            (np.array([1, 0, 0])),
        ],
    )
    def test_velocity_constant(self, vector):
        vector_generator = ConstantGenerator(constant=vector)
        fluid_generator = FluidGenerator(generator=vector_generator)
        first_fluid_vector = fluid_generator.velocity
        assert np.all(first_fluid_vector == vector)
        assert np.all(first_fluid_vector == fluid_generator.next())

    @pytest.mark.parametrize(
        "mean, cov",
        [
            (np.array([1, 2, 0]), np.array([[2, 1, 1], [1, 2, 0.9], [1, 0.9, 1]])),
            (np.array([4, 5, 3]), np.array([[3, 1, 1], [1, 3, 1], [1, 1, 2]])),
            (np.array([100, 50, 20]), np.array([[10, 5, 5], [5, 10, 4.5], [5, 4.5, 5]])),
            (np.array([120, 130, 40]), np.array([[10, 5, 1], [5, 10, 2], [1, 2, 5]])),
        ],
    )
    def test_velocity_random(self, mean, cov):
        vector_generator = MVGaussianGenerator(mean=mean, cov=cov)
        fluid_generator = FluidGenerator(vector_generator)
        first_fluid_vector = fluid_generator.velocity
        next_fluid_vector = fluid_generator.next()
        assert np.all(next_fluid_vector != first_fluid_vector)

    @pytest.mark.parametrize(
        "vector",
        [
            (np.array([1, 0, 1])),
            (np.array([0, 1, 0])),
            (np.array([-1, 0, 1])),
            (np.array([0, -1, 0])),
            (np.array([1, 1, 1])),
            (np.array([-1, -1, -1])),
        ],
    )
    def test_speed(self, vector):
        vector_generator = ConstantGenerator(constant=vector)
        fluid_generator = FluidGenerator(generator=vector_generator)
        generated_fluid_vector = fluid_generator.next()
        assert np.all(vector == generated_fluid_vector)
        assert np.linalg.norm(vector) == fluid_generator.speed

    @pytest.mark.parametrize(
        "vector, expected_direction",
        [
            (np.array([1, 0, 1]), 0),
            (np.array([0, 1, -3]), 90),
            (np.array([-1, 0, -1]), -180),
            (np.array([0, -1, 0]), -90),
            (np.array([1, 1, 4]), 45),
            (np.array([-1, -1, 6]), -135),
        ],
    )
    def test_direction(self, vector, expected_direction):
        vector_generator = ConstantGenerator(constant=vector)
        fluid_generator = FluidGenerator(generator=vector_generator)
        calculated_direction = fluid_generator.direction
        assert np.isclose(calculated_direction, expected_direction, atol=1e-7)
