from boat_simulator.common.sensors import GPS
import numpy as np


class TestGPS:
    def test_gps_init(self):
        lat_lon = np.array([1, 0])
        speed = 100
        heading = 1.09

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
        )

        assert (gps.lat_lon == lat_lon).all()
        assert gps.speed == speed
        assert gps.heading == heading

    def test_gps_read_no_noise(self):
        lat_lon = np.array([1, 0])
        speed = np.random.randint(0, 100)
        heading = np.random.rand()

        gps = GPS(lat_lon=lat_lon, speed=speed, heading=heading, enable_noise=False)

        assert np.all(gps.read("lat_lon") == lat_lon)
        assert gps.read("speed") == speed
        assert gps.read("heading") == heading

    def test_gps_gaussian_noise(self):
        lat_lon = np.array([1, 0])
        speed = np.random.randint(0, 100)
        heading = np.random.rand()
        mean = 0

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
        )

        NUM_READINGS = 10000
        speed_readings = np.zeros(NUM_READINGS)
        heading_readings = np.zeros(NUM_READINGS)
        lat_lon_readings = np.zeros(shape=(NUM_READINGS, 2))
        for i in range(NUM_READINGS):
            speed_readings[i] = gps.read("speed")
            heading_readings[i] = gps.read("heading")
            lat_lon_readings[i, :] = gps.read("lat_lon")

        for reading, init_data in zip(
            [speed_readings, heading_readings, lat_lon_readings],
            [speed, heading, lat_lon],
        ):
            sample_mean = np.mean(reading, axis=0)
            assert np.allclose(sample_mean, mean + init_data, atol=0.1)

    def test_gps_sensor_update(self):
        lat_lon = np.array([0, 0])
        speed = 0
        heading = 0

        gps = GPS(
            lat_lon=lat_lon,
            speed=speed,
            heading=heading,
        )

        NUM_READINGS = 100
        for i in range(NUM_READINGS):
            speed_reading = gps.read("speed")
            assert speed_reading == i
            gps.update(speed=i + 1)

            heading_reading = gps.read("heading")
            assert heading_reading == i
            gps.update(heading=i + 1)

            lat_lon_reading = gps.read("lat_lon")
            assert (lat_lon_reading == np.array([i, i])).all()
            gps.update(lat_lon=(lat_lon_reading + 1))

    def test_gps_sensor_update_delay(self):
        lat_lon = np.array([0, 0])
        speed0 = 0
        heading = 0

        # Initialized data is read without delay
        gps = GPS(lat_lon=lat_lon, speed=speed0, heading=heading, enable_delay=True)
        assert gps.read("speed") == speed0

        NUM_UPDATES = 3
        for i in range(NUM_UPDATES):
            gps.update(speed=(i + 1))
            assert gps.read("speed") == i
