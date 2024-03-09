import json
import random

# speed (km/h)
max_speed = 22
min_speed = 10

# angle convention (degrees)
min_direction = -180
max_direction = 180

input_file = "./tests/simulation/data/gps.json"


def modify_gps():
    with open(input_file, "r") as file:
        data = json.load(file)

    new_data = []

    for i in range(len(data)):
        new_data.append(
            {
                "latitude": data[i]["latitude"],
                "longitude": data[i]["longitude"],
                "speed": random.randint(min_speed, max_speed),
                "heading": random.randint(min_direction, max_direction)
            }
        )

    with open("./tests/simulation/data/gps.json", "w") as f:
        json.dump(new_data, f, indent=4)


def main():
    modify_gps()


if __name__ == "__main__":
    main()
