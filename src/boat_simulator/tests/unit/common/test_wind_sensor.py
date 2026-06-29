import numpy as np

from boat_simulator.common.conventions import NED, Velocity
from boat_simulator.common.sensors import SimWindSensor
from boat_simulator.common.types import Vec2


def wind(x: float, y: float) -> Vec2[Velocity, NED]:
    return Vec2.from_xy(x, y)


class TestSimWindSensor:
    def test_init_no_noise(self):
        init_wind = wind(1.0, 0.0)
        ws = SimWindSensor(wind=init_wind)
        np.testing.assert_array_equal(ws.wind.data, init_wind.data)

    def test_no_noise_returns_true_wind(self):
        init_wind = wind(3.0, -2.0)
        ws = SimWindSensor(wind=init_wind)
        for _ in range(10):
            np.testing.assert_array_equal(ws.wind.data, init_wind.data)

    def test_noise_distribution(self):
        """Noisy readings should be zero-mean with identity covariance when true wind is zero."""
        ws = SimWindSensor(wind=wind(0.0, 0.0), noise_stdev=[1.0, 1.0], enable_noise=True)
        NUM_READINGS = 10000
        readings = np.array([ws.wind.data for _ in range(NUM_READINGS)])
        assert np.allclose(readings.mean(axis=0), [0.0, 0.0], atol=0.2)
        assert np.allclose(np.cov(readings, rowvar=False), np.eye(2), atol=0.2)

    def test_setter_updates_true_wind(self):
        ws = SimWindSensor(wind=wind(0.0, 0.0))
        for i in range(5):
            np.testing.assert_array_equal(ws.wind.data, [i, i])
            ws.wind = Vec2(ws.wind.data + 1)

    def test_wind_is_2d(self):
        ws = SimWindSensor(wind=wind(1.0, 2.0))
        assert ws.wind.data.shape == (2,)
        np.testing.assert_array_equal(ws.wind.data, [1.0, 2.0])

    def test_noise_stdev_scales_variance(self):
        stdev = [2.0, 3.0]
        ws = SimWindSensor(wind=wind(0.0, 0.0), noise_stdev=stdev, enable_noise=True)
        readings = np.array([ws.wind.data for _ in range(20000)])
        expected_cov = np.diag(np.square(stdev))
        assert np.allclose(np.cov(readings, rowvar=False), expected_cov, atol=0.3)
