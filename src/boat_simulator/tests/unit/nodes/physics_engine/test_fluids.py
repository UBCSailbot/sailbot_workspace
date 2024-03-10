"""Tests classes and functions in boat_simulator/nodes/physics_engine/fluids.py"""

import numpy as np
import pytest
from boat_simulator.common.generators import ConstantGenerator, GaussianGenerator, VectorGenerator
from boat_simulator.nodes.physics_engine.fluid_generation import FluidGenerator

class TestFluidGenerator:


    @pytest.mark.parametrize("vector", [
            (np.array([1, 0])),
            (np.array([0, 1])),
            (np.array([1, 0])),
    ])

    def test_velocity_constant(self, vector):
        vector_generator = ConstantGenerator(vector)
        fluid_generator = FluidGenerator(vector_generator)
        generated_fluid_vector = fluid_generator.next()
        assert np.all(generated_fluid_vector == vector)

    def test_velocity_random(self):
        vector_generator = VectorGenerator()
        fluid_generator = FluidGenerator(vector_generator)
        # generate first vector
        generated_fluid_vector = fluid_generator.next()
        # call it again (generate a new vector), and assign to different variable
        # compare the two, they should not be equal
        assert np.all(generated_fluid_vector == vector_generator)

    @pytest.mark.parametrize("vector", [
            (np.array([1, 0])),
            (np.array([0, 1])),
            (np.array([-1, 0])),
            (np.array([0, -1])),  
            (np.array([1, 1])),  
            (np.array([-1, -1])),
    ])

    def test_speed(self, vector):
        vector_generator = ConstantGenerator(constant=vector)
        fluid_generator = FluidGenerator(generator=vector_generator)
        generated_fluid_vector = fluid_generator.next()
        assert np.all(vector == generated_fluid_vector)
        assert(np.linalg.norm(vector) == fluid_generator.speed)

    @pytest.mark.parametrize("vector, expected_direction", [
        (np.array([1, 0]), 0),  # Rightward, 0 degrees
        (np.array([0, 1]), 90),  # Upward, 90 degrees
        (np.array([-1, 0]), -180),  # Leftward, 180 or -180 degrees, normalized to 180 in this context
        (np.array([0, -1]), -90),  # Downward, -90 degrees
        (np.array([1, 1]), 45),  # Diagonal (right-upward), 45 degrees
        (np.array([-1, -1]), -135),  # Diagonal (left-downward), -135 degrees
    ])
    def test_direction(self, vector, expected_direction):
        vector_generator = ConstantGenerator(constant=vector)
        fluid_generator = FluidGenerator(generator=vector_generator)
        fluid_generator.next()  # Generate the next fluid vector to set the internal state
        calculated_direction = fluid_generator.direction
        assert np.isclose(calculated_direction, expected_direction, atol=1e-7), f"Expected direction {expected_direction}, but got {calculated_direction}"

class TestWindGenerator:
    pass