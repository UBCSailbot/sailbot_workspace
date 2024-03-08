"""Low level control logic for actuating the rudder and the sail."""

from boat_simulator.common.types import Scalar
from typing import Any, List
from abc import ABC, abstractmethod


class PID(ABC):

    """Abstract class for a PID controller.

    Attributes:
        `kp` (Scalar): The proportional component tuning constant.
        `ki` (Scalar): The integral component tuning constant.
        `kd` (Scalar): The derivative component tuning constant.
        `time_period` (Scalar): Constant time period between error samples.
        `buf_size` (int): The max number of error samples to store for integral component.
        `error_timeseries` (List[Scalar]): Timeseries of error values computed over time.
    """

    # Private class member defaults
    __kp: Scalar = 0
    __ki: Scalar = 0
    __kd: Scalar = 0
    __time_period: Scalar = 1
    __buf_size: int = 50
    __error_timeseries: List[Scalar] = list()

    def __init__(self, kp: Scalar, ki: Scalar, kd: Scalar, time_period: Scalar, buf_size: int):
        """Initializes the class attributes. Note that this class cannot be directly instantiated.

        Args:
            `kp` (Scalar): The proportional component tuning constant.
            `ki` (Scalar): The integral component tuning constant.
            `kd` (Scalar): The derivative component tuning constant.
            `time_period` (Scalar): Time period between error samples.
            `buf_size` (int): The max number of error samples to store for integral component.
        """
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        self.__buf_size = buf_size
        self.__time_period = time_period
        self.__error_timeseries = list()

    def step(self, current: Any, target: Any) -> Scalar:
        """Computes the correction factor.

        Args:
            `current` (Any): Current state of the system.
            `target` (Any): Target state of the system.

        Returns:
            Scalar: Correction factor.
        """
        raise NotImplementedError()

    def reset(self, is_latest_error_kept: bool = False) -> None:
        """Empties the error timeseries of the PID controller, effectively starting a new
        control iteration.

        Args:
            is_latest_error_kept (bool, optional): True if the latest error is kept in the error
                timeseries to avoid starting from scratch if the target remains the same. False
                if the timeseries should be completely emptied. Defaults to False.
        """
        raise NotImplementedError()

    def __append_error(self, error: Scalar) -> None:
        """Appends the latest error to the error timeseries attribute. If the timeseries is at
        the maximum buffer size, the least recently computed error is evicted from the timeseries
        and the new one is appended.

        Args:
            `error` (Scalar): The latest error.
        """
        raise NotImplementedError()

    @abstractmethod
    def _compute_error(self, current: Any, target: Any) -> Scalar:
        """Computes the currently observed error.

        Args:
            current (Any): Current state of the system.
            target (Any): Target state of the system.

        Returns:
            Scalar: Current error between the current and target states.
        """
        pass

    @abstractmethod
    def _compute_proportional_response(self) -> Scalar:
        """
        Returns:
            Scalar: The proportional component of the correction factor.
        """
        pass

    @abstractmethod
    def _compute_integral_response(self) -> Scalar:
        """
        Returns:
            Scalar: The integral component of the correction factor.
        """
        pass

    @abstractmethod
    def _compute_derivative_response(self) -> Scalar:
        """
        Returns:
            Scalar: The derivative component of the correction factor.
        """
        pass

    @property
    def kp(self) -> Scalar:
        return self.__kp

    @property
    def ki(self) -> Scalar:
        return self.__ki

    @property
    def kd(self) -> Scalar:
        return self.__kd

    @property
    def buf_size(self) -> Scalar:
        return self.__buf_size

    @property
    def time_period(self) -> Scalar:
        return self.__time_period

    @property
    def error_timeseries(self) -> List[Scalar]:
        return self.__error_timeseries


class RudderPID(PID):
    """Class for the rudder PID controller.

    Extends: PID
    """

    def __init__(self, kp: Scalar, ki: Scalar, kd: Scalar, time_period: Scalar, buf_size: int):
        """Initializes the class attributes.

        Args:
            `kp` (Scalar): The proportional component tuning constant.
            `ki` (Scalar): The integral component tuning constant.
            `kd` (Scalar): The derivative component tuning constant.
            `time_period` (Scalar): Time period between error samples.
            `buf_size` (int): The max number of error samples to store for integral component.
        """
        super().__init__(kp, ki, kd, time_period, buf_size)

    def _compute_error(self, current: Scalar, target: Scalar) -> Scalar:
        raise NotImplementedError()

    def _compute_proportional_response(self) -> Scalar:
        raise NotImplementedError()

    def _compute_integral_response(self) -> Scalar:
        raise NotImplementedError()

    def _compute_derivative_response(self) -> Scalar:
        raise NotImplementedError()
