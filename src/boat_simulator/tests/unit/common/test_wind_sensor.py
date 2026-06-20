import numpy as np

from boat_simulator.common.sensors import SimWindSensor


class TestSimWindSensor:
    def test_init_no_noise(self):
        init_wind = np.array([1.0, 0.0])
        ws = SimWindSensor(wind=init_wind)
        assert np.all(ws.wind == init_wind)

    def test_no_noise_returns_true_wind(self):
        init_wind = np.array([3.0, -2.0])
        ws = SimWindSensor(wind=init_wind)
        for _ in range(10):
            assert np.all(ws.wind == init_wind)

    def test_noise_distribution(self):
        """Noisy readings should be zero-mean with identity covariance when true wind is zero."""
        ws = SimWindSensor(wind=np.zeros(2), noise_stdev=[1.0, 1.0], enable_noise=True)
        NUM_READINGS = 10000
        readings = np.array([ws.wind for _ in range(NUM_READINGS)])
        assert np.allclose(readings.mean(axis=0), [0.0, 0.0], atol=0.2)
        assert np.allclose(np.cov(readings, rowvar=False), np.eye(2), atol=0.2)

    def test_setter_updates_true_wind(self):
        ws = SimWindSensor(wind=np.zeros(2))
        for i in range(5):
            assert np.all(ws.wind == [i, i])
            ws.wind = ws.wind + 1

    def test_wind_sliced_to_2d(self):
        """A 3D input vector should be truncated to the first two components."""
        ws = SimWindSensor(wind=np.array([1.0, 2.0, 99.0]))
        assert ws.wind.shape == (2,)
        assert np.all(ws.wind == [1.0, 2.0])

    def test_noise_stdev_scales_variance(self):
        stdev = [2.0, 3.0]
        ws = SimWindSensor(wind=np.zeros(2), noise_stdev=stdev, enable_noise=True)
        readings = np.array([ws.wind for _ in range(20000)])
        expected_cov = np.diag(np.square(stdev))
        assert np.allclose(np.cov(readings, rowvar=False), expected_cov, atol=0.3)
