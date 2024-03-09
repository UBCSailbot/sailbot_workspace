"""Random vector generator classes."""

from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.types import Scalar, ScalarOrArray


class VectorGenerator(ABC):
    """This class's purpose is to generate values in a sequence. It acts as a base class for other
    generators.

    Attributes:
        seed (int): The seed used to seed the random number generator.
        rng (np.random.Generator): The seeded random number generator.
    """

    def __init__(self, seed: int = 0):
        """Initializes an instance of `VectorGenerator`. Note that this class cannot be
        instantiated directly since it is abstract.

        Args:
            seed (int, optional): The seed used to seed the random number generator
                (if used at all). Defaults to 0.
        """
        self.__seed = seed
        self.__rng = np.random.default_rng(seed=seed)

    def next(self) -> ScalarOrArray:
        """Generates the next value in the sequence. This function acts as an alias to the
        function _next().

        Returns:
            ScalarOrArray: Generated value.
        """
        return self._next()

    @abstractmethod
    def _next(self) -> ScalarOrArray:
        """Generates the next value in the sequence.

        Returns:
            ScalarOrArray: Generated array.
        """
        pass

    @property
    def seed(self) -> int:
        return self.__seed

    @property
    def rng(self) -> np.random.Generator:
        return self.__rng


class GaussianGenerator(VectorGenerator):
    """This class generates random scalars using a univariate gaussian distribution.

    Attributes:
        mean (Scalar): The mean of the gaussian distribution.
        stdev (Scalar): The standard deviation of the gaussian distribution. This value is
            strictly positive.
        value (Scalar): The latest generated scalar.

    Extends: VectorGenerator
    """

    def __init__(self, mean: Scalar, stdev: Scalar, seed: int = 0):
        """Initializes an instance of GaussianGenerator.

        Args:
            mean (Scalar): _description_
            stdev (Scalar): _description_
            seed (int, optional): _description_. Defaults to 0.
        """
        super().__init__(seed=seed)
        self.__mean = mean
        self.__stdev = stdev
        self.next()

    def _next(self) -> Scalar:
        self.__value = np.random.normal(self.mean, self.stdev)
        return self.__value

    @property
    def mean(self) -> Scalar:
        return self.__mean

    @property
    def stdev(self) -> Scalar:
        return self.__stdev

    @property
    def value(self) -> Scalar:
        return self.__value


class MVGaussianGenerator(VectorGenerator):
    """This class generates random vectors using a multivariate gaussian distribution.

    Attributes:
        mean (NDArray): The mean of the gaussian distribution. Shape should be (N,).
        cov (NDArray): The covariance matrix of the gaussian distribution. Should be positive
            semi-definite and have a shape of (N,N).
        value (NDArray): The latest generated array.

    Extends: VectorGenerator
    """

    def __init__(self, mean: NDArray, cov: NDArray, seed: int = 0):
        """Initializes an instance of MVGaussianGenerator.

        Args:
            mean (NDArray): The mean of the gaussian distribution. Shape should be (N,).
            cov (NDArray): The covariance matrix of the gaussian distribution. Should be positive
                semi-definite and have a shape of (N,N).
            seed (int, optional): The seed that seeds the random number generator. Defaults to 0.
        """
        super().__init__(seed=seed)
        self.__mean = mean
        self.__cov = cov
        self.next()

    def _next(self) -> NDArray:
        self.__value = self.rng.multivariate_normal(self.mean, self.cov)
        return self.__value

    @property
    def mean(self) -> NDArray:
        return self.__mean

    @property
    def cov(self) -> NDArray:
        return self.__cov

    @property
    def value(self) -> NDArray:
        return self.__value


class ConstantGenerator(VectorGenerator):
    """This class returns the same specified value when asked to generate a new value.

    Attributes:
        constant (ScalarOrArray): The constant value to return upon generation. It can
            either be a scalar or an array.

    Extends: VectorGenerator
    """

    def __init__(self, constant: ScalarOrArray):
        """Initializes an instance of ConstantGenerator.

        Args:
            constant (ScalarOrArray): The constant value to return upon generation. It can
                either be a scalar or an array.
        """
        super().__init__(seed=0)
        self.__constant = constant

    def _next(self) -> ScalarOrArray:
        return self.__constant

    @property
    def constant(self) -> ScalarOrArray:
        return self.__constant
