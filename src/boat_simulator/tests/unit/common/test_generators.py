"""Tests classes and functions in boat_simulator/common/generators.py"""

import numpy as np
import pytest


from boat_simulator.common.generators import (
    ConstantGenerator,
    GaussianGenerator,
    MVGaussianGenerator,
)


class TestGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, stdev, threshold",
        [(1.0, 1.0, 0.2), (10, 0.0, 0.0), (-1.0, 0.0, 0.2), (4.2, 5.1, 0.2), (120.0, 120.0, 10.0)],
    )
    def test_gaussian_generator(self, mean: float, stdev: float, threshold):
        """
        This test compares the mean and standard deviation computed from an array
        of generated sequence of scalars originating from the GaussianGenerator to the expected

        Args:
            mean (float): Describes the mean of the Gaussian distribution used
            by the GaussianGenerator.
            stdev (float): Describes the standard deviation of the Gaussian distribution
            used by the GaussianGenerator.
            threshold (float): Threshold allowed between expected mean and standard deviation
            to computed mean and standard deviation.
        """
        NUM_SAMPLES = 50000

        samples = np.zeros(NUM_SAMPLES)

        generator = GaussianGenerator(mean=mean, stdev=stdev)

        for i in range(NUM_SAMPLES):
            samples[i] = generator.next()

        sample_mean = np.mean(samples)
        sample_std = np.std(samples)

        assert np.allclose(sample_std, stdev, atol=threshold)
        assert np.allclose(sample_mean, mean, atol=threshold)


class TestMVGaussianGenerator:
    @pytest.mark.parametrize(
        "mean, cov",
        [
            (np.array([1, 1]), np.eye(2)),
            (np.array([1, 2]), np.array([[2, 1], [1, 2]])),
            (np.array([4, 5]), np.array([[3, 1], [1, 3]])),
            (np.array([100, 50]), np.array([[10, 5], [5, 10]])),
            (np.array([120, 130]), np.array([[10, 5], [5, 10]])),
            (np.array([1, 1, 1]), np.eye(3)),
            (np.array([1, 2, 3]), np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]])),
            (np.array([4, 5, 6]), np.array([[3, 2, 2], [2, 3, 2], [2, 2, 3]])),
        ],
    )
    def test_multivariate_vector_generation_2d(self, mean, cov):
        """This test compares the calculated cov/mean of generated vectors against
        expected mean/cov arrays

        Args:
            mean (array): Input array of size n with average 1
            cov (array): Identity matrix based on mean array size (I_n)
        """
        NUM_SAMPLES = 10000
        samples = np.zeros(shape=(NUM_SAMPLES, mean.size))
        generator = MVGaussianGenerator(mean=mean, cov=cov)
        for i in range(NUM_SAMPLES):
            samples[i, :] = generator.next()
        sample_mean = np.mean(samples, axis=0)
        sample_cov = np.cov(samples, rowvar=False)

        assert np.allclose(sample_cov, cov, atol=0.2)
        assert np.isclose(sample_mean, mean, 0.1).all()


class TestConstantGenerator:
    @pytest.mark.parametrize(
        "constant",
        [
            50.22,
            10111,
            12.3456789,
            (np.array([1]),),
            (np.array([1, 2.2568]),),
            (np.array([1 * 7 / 6, 2.75, 3]),),
            (np.array([[1.0001, 1], [1, 1.5674]]),),
            (np.array([[1091237, 12319.6], [784520, 1]]),),
            (np.array([[1.3333, 2.6666, 3.9999], [3.0001, 2, 1]]),),
        ],
    )
    def test_constant_vector_generator(self, constant):
        """This test compares if the generated vector is exactly
        the same as the initial vector

        Args:
            constant (array): The constant array to return upon array generation.
        """
        generator = ConstantGenerator(constant=constant)
        samples = generator.next()
        assert np.isclose(constant, samples, 0.1).all()
