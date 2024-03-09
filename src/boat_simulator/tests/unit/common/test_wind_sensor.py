from boat_simulator.common.sensors import WindSensor
import numpy as np
from boat_simulator.common.generators import (
    MVGaussianGenerator,
    ConstantGenerator,
)


class TestWindSensor:
    def test_wind_sensor_init(self):
        init_data = np.array([1, 0])
        error_fn = None
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        assert ws.wind_noisemaker == error_fn
        assert np.all(ws.wind == init_data)

    def test_wind_sensor_init_implicit_error_fn(self):
        init_data = np.array([1, 0])
        ws = WindSensor(wind=init_data)

        assert ws.wind_noisemaker is None
        assert np.all(ws.wind == init_data)

    def test_wind_sensor_read_no_error(self):
        init_data = np.array([1, 0])
        ws = WindSensor(
            wind=init_data,
        )
        read_data = ws.read("wind")
        assert (init_data == read_data).all()

    def test_wind_sensor_read_constant_error(self):
        init_data = np.array([1, 0])
        const_err = 0.1
        error_fn = ConstantGenerator(constant=0.1)
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        read_data = ws.read("wind")
        assert ((init_data + const_err) == read_data).all()

    def test_wind_sensor_read_mv_gaussian_error(self):
        init_data = np.array([1, 0])
        mean = np.array([1, 1])
        cov = np.eye(2)
        error_fn = MVGaussianGenerator(mean=mean, cov=cov)
        ws = WindSensor(
            wind=init_data,
            wind_noisemaker=error_fn,
        )

        NUM_READINGS = 10000
        reading = np.zeros(shape=(NUM_READINGS, mean.size))
        for i in range(NUM_READINGS):
            reading[i, :] = ws.read("wind")

        sample_mean = np.mean(reading, axis=0)
        sample_cov = np.cov(reading, rowvar=False)

        assert np.allclose(sample_cov, cov, atol=0.2)
        assert np.isclose(sample_mean, mean + init_data, 0.1).all()

    def test_wind_sensor_update(self):
        init_data = np.zeros(2)
        ws = WindSensor(wind=init_data)

        NUM_READINGS = 100
        for i in range(NUM_READINGS):
            wind = ws.read("wind")
            assert (wind == np.array([i, i])).all()
            ws.update(wind=(wind + 1))
