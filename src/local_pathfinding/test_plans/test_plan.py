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

        # TODO continue with GPS, wind sensor, and global path data

    @property
    def land(self):
        return self._land

    @property
    def ais(self):
        return self._ais
