import os

import custom_interfaces.msg as ci
import yaml
from shapely.geometry import MultiPolygon, Polygon


class TestPlan:
    _instance = None

    """An immutable singleton collection of data defining a local pathfinding test plan."""

    def __new__(cls, file_name: str):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, file_name: str):
        if getattr(self, "_created", False):
            return
        self._created = True

        _, ext = os.path.splitext(file_name)
        if ext.lower() not in [".yaml", ".yml"]:
            raise ValueError(f"Unsupported test plan file extension: {ext}")

        with open(os.path.join(os.path.dirname(__file__), file_name), "r") as file:
            data = yaml.safe_load(file)

        self._land = MultiPolygon([Polygon(p) for p in data.get("land", [])])
        self._ais = [
            ci.HelperAISShip(
                id=ship["id"],
                lat_lon=ci.HelperLatLon(latitude=ship["lat"], longitude=ship["lon"]),
                cog=ci.HelperHeading(heading=ship["heading"]),
                sog=ci.HelperSpeed(speed=ship["speed"]),
                rot=ci.HelperROT(rot=ship["rot"]),
                width=ci.HelperDimension(dimension=ship["width"]),
                length=ci.HelperDimension(dimension=ship["height"]),
            )
            for ship in data.get("ais", [])
        ]

        sailbot_state = data.get("sailbot_state", None)

        if sailbot_state is None:
            raise ValueError("Sailbot state is required in the test plan.")

        self._heading_deg = int(sailbot_state.get("heading", 0.0))
        self._speed = ci.HelperSpeed(speed=float(sailbot_state.get("speed", 0.0)))
        self._current_location = ci.HelperLatLon(
            latitude=float(sailbot_state.get("latitude", 0.0)),
            longitude=float(sailbot_state.get("longitude", 0.0)),
        )

        # Wind sensor data
        wind_sensor = sailbot_state.get("wind_sensor", None)

        if wind_sensor is None:
            raise ValueError("Wind sensor data is required in the test plan.")

        self._tw_speed_kmph = float(wind_sensor.get("tw_speed_kmph", 0.0))
        self._tw_dir_deg = int(wind_sensor.get("tw_dir_deg", 0))

        # Global Path data
        global_path = data.get("global_path", None)
        path = ci.Path()

        if global_path is None:
            raise ValueError("Global path data is required in the test plan.")

        for waypoint in global_path:

            latitude = float(waypoint.get("latitude", 0.0))
            longitude = float(waypoint.get("longitude", 0.0))

            path.waypoints.append(ci.HelperLatLon(latitude=latitude, longitude=longitude))

        self._global_path = path

    @property
    def land(self):
        return self._land

    @property
    def ais(self):
        return self._ais

    @property
    def wind_data(self):
        return self._tw_speed_kmph, self._tw_dir_deg

    @property
    def sailbot_state(self):
        return self._heading_deg, self._speed, self._current_location

    @property
    def global_path(self):
        return self._global_path
