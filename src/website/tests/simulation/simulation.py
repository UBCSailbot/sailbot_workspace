"""
Pathfinding simulation for the website, using GPS, local path, global path, and AIS.

Requirements:
- Ensure the DB is cleared and running.
- Ensure the env var 'MONGODB_URI' in the file '.env.local' is connected to correct the database name.
- Ensure the env var 'NEXT_PUBLIC_POLLING_TIME_MS' is set to 500 in the file '.env.local'.
"""

import json
import time
from datetime import datetime, timedelta

import pymongo

CONNECTION_STRING = "mongodb://localhost:27017"
DATABASE_NAME = "sailbot_db"
TIME_INTERVAL_M = 5

# Load the database
client = pymongo.MongoClient(CONNECTION_STRING)
db = client[DATABASE_NAME]


def read_json_file(file_name):
    with open(file_name, "r") as file:
        data = json.load(file)
    return data


# Load all data files
gps_data = read_json_file("./data/gps.json")
local_path_data = read_json_file("./data/localpath.json")
global_path_data = read_json_file("./data/globalpath.json")
ais_ships_data = read_json_file("./data/aisships.json")
batteries_data = read_json_file("./data/batteries.json")
wind_sensors_data = read_json_file("./data/wind_sensors.json")

# Load all database collections
gps = db["gps"]
local_path = db["localpaths"]
global_path = db["globalpaths"]
ais_ships = db["aisships"]
batteries = db["batteries"]
wind_sensors = db["windsensors"]


def write_to_mongodb(data, collection, i):
    timestamp = datetime.now() + timedelta(minutes=i*TIME_INTERVAL_M)
    data["timestamp"] = timestamp.isoformat()
    collection.insert_one(data)
    print(f"Data written to MongoDB collection '{collection.name}'")


def clear_mongodb_collection(collection):
    collection.delete_many({})
    print(f"Cleared data in MongoDB collection '{collection.name}'")


def preload_data():
    print("\nPreloading Data...\n")
    write_to_mongodb(global_path_data[0], global_path, 0)
    write_to_mongodb(local_path_data[0], local_path, 0)
    write_to_mongodb(ais_ships_data[0], ais_ships, 0)
    print("\nDone\n")


def clear():
    print("\nClearing all collections:\n")
    clear_mongodb_collection(gps)
    clear_mongodb_collection(local_path)
    clear_mongodb_collection(global_path)
    clear_mongodb_collection(ais_ships)
    clear_mongodb_collection(batteries)
    clear_mongodb_collection(wind_sensors)
    print("\nCleared all collections\n")


def display_help():
    print("\nAvailable options:")
    print("clear - Clears the MongoDB database.")
    print(
        "preload - Writes all local path, global path, and ais ships data into the database to prepare the simulation."
    )
    print("start - Starts the simulation on the website by periodically updating the gps.")
    print(
        "restart - Restarts the simulation on the website. Automatically clears and preloads the data into the database."
    )
    print("exit - Exits the script.\n")


while True:
    user_input = input("Enter your command: ")

    if user_input.lower() == "preload":
        preload_data()
    elif user_input.lower() == "clear":
        clear()
    elif user_input.lower() == "restart":
        clear()
        preload_data()
        j = 0
        time.sleep(2)
        for i in range(1, len(gps_data)):
            write_to_mongodb(batteries_data[i], batteries, i)
            write_to_mongodb(wind_sensors_data[i], wind_sensors, i)
            write_to_mongodb(gps_data[i], gps, i)
            lp_len = len(local_path_data[j]["waypoints"]) - 1
            if (
                local_path_data[j]["waypoints"][lp_len]["latitude"] == gps_data[i]["latitude"]
                and local_path_data[j]["waypoints"][lp_len]["longitude"]
                == gps_data[i]["longitude"]
            ):
                time.sleep(1)
                j += 1
                if j < len(local_path_data):
                    write_to_mongodb(local_path_data[j], local_path, i)
            time.sleep(1)
    elif user_input.lower() == "start":
        j = 0
        time.sleep(2)
        for i in range(1, len(gps_data)):
            write_to_mongodb(batteries_data[i], batteries, i)
            write_to_mongodb(wind_sensors_data[i], wind_sensors, i)
            write_to_mongodb(gps_data[i], gps, i)
            if (
                local_path_data[j]["waypoints"][1]["latitude"] == gps_data[i]["latitude"]
                and local_path_data[j]["waypoints"][1]["longitude"] == gps_data[i]["longitude"]
            ):
                time.sleep(1)
                j += 1
                write_to_mongodb(local_path_data[j], local_path, i)
            time.sleep(1)
    elif user_input.lower() == "exit":
        break
    elif user_input.lower() == "help":
        display_help()
    else:
        print("\nInvalid input. Type 'help' for assistance.\n")
