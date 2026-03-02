import os

import yaml
from shapely.geometry import MultiPolygon, Polygon

import custom_interfaces.msg as ci


class TestPlan:
    _instance = None

    """An immutable singleton collection of data defining a local pathfinding test plan.

    The choice to make the class immutable is just for simplicity's sake, it gets passed around a
    lot so it's good to know nothing modifies it anywhere.

    The choice to make it a singleton is to ensure file system I/O is done at most once, to load
    the test plan from yaml, and then never again for the duration of the test.

    """

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

        try:
            with open(os.path.join(os.path.dirname(__file__), file_name), "r") as file:
                data = yaml.safe_load(file)
        except Exception as e:
            raise RuntimeError(f"Failed to load test plan file '{file_name}': {e}")

        self._land = MultiPolygon([Polygon(points) for points in data.get("land", [])])
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
        self._gps = ci.GPS(
            lat_lon=ci.HelperLatLon(
                latitude=data["gps"]["latitude"], longitude=data["gps"]["longitude"]
            ),
            speed=ci.HelperSpeed(speed=data["gps"]["speed_kmph"]),
            heading=ci.HelperHeading(heading=data["gps"]["heading_deg"]),
        )
        self._tw_speed_kmph = data["tw_speed_kmph"]
        self._tw_dir_deg = data["tw_dir_deg"]

        self._global_path = ci.Path(
            waypoints=[
                ci.HelperLatLon(latitude=wp["latitude"], longitude=wp["longitude"])
                for wp in data["global_path"]["waypoints"]
            ]
        )

    @property
    def global_path(self):
        return self._global_path

    @property
    def gps(self):
        return self._gps

    @property
    def ais(self):
        return self._ais

    @property
    def land(self):
        return self._land

    @property
    def tw_speed_kmph(self):
        return self._tw_speed_kmph

    @property
    def tw_dir_deg(self):
        return self._tw_dir_deg
