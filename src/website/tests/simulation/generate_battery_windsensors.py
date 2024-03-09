import json
import random

# Constants
no_of_objects = 36

# voltage (volts)
max_voltage = 10
min_voltage = 5

# current (amperes)
max_current = 5
min_current = 3

# speed (km/h)
max_speed = 22
min_speed = 10

# angle convention (degrees)
min_direction = -180
max_direction = 180


def write_battery_json():
    battery_data = []

    for i in range(no_of_objects):
        battery_data.append(
            {
                "batteries": [
                    {
                        "voltage": random.randint(min_voltage, max_voltage),
                        "current": random.randint(min_current, max_current),
                    },
                    {
                        "voltage": random.randint(min_voltage, max_voltage),
                        "current": random.randint(min_current, max_current),
                    },
                ]
            }
        )

    with open("./tests/simulation/data/batteries.json", "w") as f:
        json.dump(battery_data, f, indent=4)


def write_wind_sensors_json():
    wind_sensors_data = []

    for i in range(no_of_objects):
        wind_sensors_data.append(
            {
                "windSensors": [
                    {
                        "speed": random.randint(min_speed, max_speed),
                        "direction": random.randint(min_direction, max_direction),
                    },
                    {
                        "speed": random.randint(min_speed, max_speed),
                        "direction": random.randint(min_direction, max_direction),
                    },
                ]
            }
        )

    with open("./tests/simulation/data/wind_sensors.json", "w") as f:
        json.dump(wind_sensors_data, f, indent=4)


def main():
    write_battery_json()
    write_wind_sensors_json()


if __name__ == "__main__":
    main()
