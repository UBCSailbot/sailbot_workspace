from boat_simulator.common.sensors import WindSensor
import numpy as np


class TestWindSensor:
    def test_wind_sensor_init(self):
        init_data = np.array([1, 0])
        ws = WindSensor(
            wind=init_data,
        )

        assert np.all(ws.wind == init_data)

    def test_wind_sensor_read_no_noise(self):
        init_data = np.array([1, 0])
        ws = WindSensor(
            wind=init_data,
        )
        read_data = ws.read("wind")
        assert np.all(init_data == read_data)

    def test_wind_sensor_read_mv_gaussian_noise(self):
        init_data = np.array([0, 0])
        mean = np.array([0, 0])
        cov = np.eye(2)
        ws = WindSensor(wind=init_data, enable_noise=True)

        NUM_READINGS = 10000
        reading = np.zeros(shape=(NUM_READINGS, mean.size))
        for i in range(NUM_READINGS):
            reading[i, :] = ws.read("wind")

        sample_mean = np.mean(reading, axis=0)
        sample_cov = np.cov(reading, rowvar=False)

        assert np.allclose(sample_mean, mean, atol=0.2)
        assert np.allclose(sample_cov, cov, atol=0.2)

    def test_wind_sensor_update_no_delay(self):
        init_data = np.array([0, 0])
        ws = WindSensor(wind=init_data)

        NUM_READINGS = 100
        for i in range(NUM_READINGS):
            wind = ws.read("wind")
            assert np.all(wind == np.array([i, i]))
            ws.update(wind=(wind + 1))

    def test_wind_sensor_update_with_delay(self):
        """
        Attempt to constantly update wind sensor with new data.
        Delay causes new data to be read in the next update cycle.
        """

        init_data = np.array([0, 0])

        ws = WindSensor(wind=init_data, enable_delay=True)

        wind = ws.read("wind")
        # Initialized data is read without delay
        assert np.all(wind == init_data)

        NUM_UPDATES = 3
        for i in range(NUM_UPDATES):
            ws.update(wind=np.array([i + 1, i + 1]))
            wind = ws.read("wind")
            assert np.all(wind == [i, i])
