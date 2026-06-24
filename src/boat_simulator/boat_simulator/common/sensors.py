from typing import Any, Sequence

import numpy as np
from numpy.typing import NDArray

from boat_simulator.common.angle_conventions import Heading
from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.generators import (
    GaussianGenerator,
    MVGaussianGenerator,
)
from boat_simulator.common.types import Vec2


class Sensor:
    """

    Interface for sensors in the Boat Simulation.

    Data delay and noise models are supported.

    Delay model will delay sensor value updates by one update cycle:
        - Sensor has initial data x0 at t = 0
        - Sensor provided new data x_1 at t = 1
        - Sensor provided new data x_2 at t = 2. At t = 2, x1 is registered into the sensor.
        - Sensor provided new data x_i at t = i. At t = i, x_{i-1} is registered into the sensor.

    Noise model will add noise to sensor values drawn from a
    Gaussian or Multi-variate Gaussian distribution.
    """

    def __init__(self, enable_delay: bool = False, enable_noise: bool = False) -> None:
        """

        Args:
            enable_noise (bool): Enables noise for fields. False by default.
            enable_delay (bool): Enables delay for fields. False by default.
        """

        self.enable_delay = enable_delay
        self.enable_noise = enable_noise

    def update(self, **kwargs):
        """
        Update attributes in Sensor using keyword arguments.

        Usage: Sensor.update(attr1=val1, attr2=val2, ...)

        Raises:
            ValueError: If kwarg is not a defined attribute in Sensor
        """

        for attr_name, attr_val in kwargs.items():
            if attr_name in self.__annotations__:
                setattr(self, attr_name, attr_val)
            else:
                raise ValueError(f"{attr_name} not a property in {self.__class__.__name__} \
                    expected one of {self.__annotations__}")

    def read(self, key: str) -> Any:
        """
        Read the value from an attribute in Sensor.

        Args:
            key (str): Attribute name to read from

        Raises:
            ValueError: If key is not an a defined attribute in Sensor

        Returns:
            Any: Value stored in attribute with name supplied in "key" argument
        """
        if key in self.__annotations__:
            return getattr(self, key)
        else:
            raise ValueError(f"{key} not a property in {self.__class__.__name__}. \
                Available keys: {self.__annotations__}")


class SimWindSensor:
    """Simulates a single wind sensor that adds Gaussian measurement noise to true wind.

    True wind is a 2D velocity vector (x, y) in km/h. Each access to `.wind` returns a
    freshly-sampled noisy reading when noise is enabled, so callers can gather statistics
    over many reads without needing to reset the true wind.
    """

    def __init__(
        self,
        wind: Vec2[Velocity, NED],
        noise_stdev: Sequence[float] = (1.0, 1.0),
        enable_noise: bool = False,
    ) -> None:
        self._enable_noise = enable_noise
        self._noise_gen = MVGaussianGenerator(
            mean=np.zeros(2), cov=np.diag(np.square(noise_stdev))
        )
        self._true_wind = wind

    @property
    def wind(self) -> Vec2[Velocity, NED]:
        """Returns the sensor reading: true wind plus Gaussian noise if enabled."""
        noise: Vec2[Velocity, NED] = Vec2(
            self._noise_gen.next() if self._enable_noise else np.zeros(2)
        )
        return self._true_wind + noise

    @wind.setter
    def wind(self, wind: Vec2[Velocity, NED]) -> None:
        """Updates the true wind vector."""
        self._true_wind = wind


class SimGPS(Sensor):
    """
    Abstraction for GPS.

    Properties:
        lat_lon (NDArray): Boat latitude and longitude in degrees [°] (2x1 array,
            [latitude, longitude]).
        speed (float): Boat speed in meters per second [m/s].
        heading (Heading): Boat heading as a Heading value object (radians,
            normalized to [-pi, pi), 0 is straight, increasing CCW).
        enable_noise (bool): Enables noise for fields. False by default.
        enable_delay (bool): Enables delay for fields. False by default.
    """

    lat_lon: NDArray
    speed: float
    heading: Heading

    def __init__(
        self,
        lat_lon: NDArray,
        speed: float,
        heading: Heading,
        lat_lon_noise_stdev: float = 0.00009,
        speed_noise_stdev: float = 0.1,
        heading_noise_stdev: float = 0.1,  # radians, matching the Heading value object
        enable_noise: bool = False,
        enable_delay: bool = False,
    ):
        super().__init__(enable_noise=enable_noise, enable_delay=enable_delay)
        self._lat_lon = lat_lon
        self._speed = speed
        self._heading = heading

        # TODO: Refactor the initialization of data fields and their respective delay controls.
        # Warning: this is not easy!

        # Delay Controls
        self.lat_lon_queue_next: bool = False
        self.lat_lon_next_value: NDArray = lat_lon

        self.speed_queue_next: bool = False
        self.speed_next_value: float = speed

        self.heading_queue_next: bool = False
        self.heading_next_value: Heading = heading

        self.lat_lon_noisemaker: GaussianGenerator = GaussianGenerator(
            mean=0, stdev=lat_lon_noise_stdev
        )
        self.speed_noisemaker: GaussianGenerator = GaussianGenerator(
            mean=0, stdev=speed_noise_stdev
        )
        self.heading_noisemaker: GaussianGenerator = GaussianGenerator(
            mean=0, stdev=heading_noise_stdev
        )

    @property  # type: ignore
    def lat_lon(self) -> NDArray:
        return (
            self._lat_lon + self.lat_lon_noisemaker.next() if self.enable_noise else self._lat_lon
        )

    @lat_lon.setter
    def lat_lon(self, lat_lon: NDArray):

        if not self.enable_delay:
            self._lat_lon = lat_lon
            return

        if self.lat_lon_queue_next:
            self._lat_lon = self.lat_lon_next_value
        else:
            self.lat_lon_queue_next = True

        self.lat_lon_next_value = lat_lon

    @property  # type: ignore
    def speed(self) -> float:
        return (
            self._speed + self.speed_noisemaker.next()  # type: ignore
            if self.enable_noise
            else self._speed
        )

    @speed.setter
    def speed(self, speed: float):

        if not self.enable_delay:
            self._speed = speed
            return

        if self.speed_queue_next:
            self._speed = self.speed_next_value
        else:
            self.speed_queue_next = True

        self.speed_next_value = speed

    @property  # type: ignore
    def heading(self) -> Heading:
        if not self.enable_noise:
            return self._heading
        # Apply noise in radians and let Heading re-wrap the result into [-pi, pi).
        return Heading(self._heading.radians + float(self.heading_noisemaker.next()))

    @heading.setter
    def heading(self, heading: Heading):

        if not self.enable_delay:
            self._heading = heading
            return

        if self.heading_queue_next:
            self._heading = self.heading_next_value
        else:
            self.heading_queue_next = True

        self.heading_next_value = heading
