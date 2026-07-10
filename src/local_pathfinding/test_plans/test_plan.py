import os
from dataclasses import dataclass
from typing import Any

import yaml
from shapely.geometry import MultiPolygon, Polygon

import custom_interfaces.msg as ci

MAX_WIND_SPEED_KMPH = 150.0
MAX_SHIP_SPEED_KMPH = 200.0
MAX_DRIFT_SPEED_KMPH = 20.0
MAX_DRIFT_ACCEL_KMPH2 = 5.0
MAX_ROT_DEG_PER_MIN = 720.0
MAX_DIMENSION_M = 500.0
MIN_LATITUDE, MAX_LATITUDE = -90.0, 90.0
MIN_LONGITUDE, MAX_LONGITUDE = -180.0, 180.0

_VALID_MOCK_NAMES = frozenset(
    {"mock_wind_sensor", "mock_gps", "mock_ais", "mock_global_path"}
)
_WIND_EVENT_FIELDS = frozenset({"timestamp", "direction_deg", "speed_kmph"})
_GPS_EVENT_FIELDS = frozenset(
    {
        "timestamp",
        "use_gps_noise",
        "use_ocean_drift",
        "use_drift_randomization",
        "ocean_drift_speed_kmph",
        "ocean_drift_dir_deg",
        "ocean_drift_accel_kmph2",
    }
)
_AIS_EVENT_FIELDS = frozenset({"timestamp", "ships"})
_AIS_SHIP_FIELDS = frozenset(
    {"id", "lat", "lon", "heading", "speed", "rot", "width", "height"}
)
_GLOBAL_PATH_EVENT_FIELDS = frozenset({"timestamp", "waypoints"})
_WAYPOINT_FIELDS = frozenset({"latitude", "longitude"})


@dataclass(frozen=True)
class WindEvent:
    timestamp: float
    direction_deg: int
    speed_kmph: float


@dataclass(frozen=True)
class GpsEvent:
    timestamp: float
    use_gps_noise: bool | None = None
    use_ocean_drift: bool | None = None
    use_drift_randomization: bool | None = None
    ocean_drift_speed_kmph: float | None = None
    ocean_drift_dir_deg: float | None = None
    ocean_drift_accel_kmph2: float | None = None


@dataclass(frozen=True)
class AisEvent:
    timestamp: float
    ships: tuple[ci.HelperAISShip, ...]


@dataclass(frozen=True)
class GlobalPathEvent:
    timestamp: float
    waypoints: tuple[ci.HelperLatLon, ...]


class TestPlan:
    """An immutable singleton collection of data defining a local pathfinding test plan.

    The choice to make the class immutable is just for simplicity's sake, it gets passed around a
    lot so it's good to know nothing modifies it anywhere.

    The choice to make it a singleton is to ensure file system I/O is done at most once, to load
    the test plan from yaml, and then never again for the duration of the test.

    """

    __test__ = False
    _instance = None

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

        if data.get("land") is not None:
            self._land = MultiPolygon([Polygon(points) for points in data["land"]])
        else:
            self._land = None

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

        if data.get("gps") is not None:
            self._gps = ci.GPS(
                lat_lon=ci.HelperLatLon(
                    latitude=data["gps"]["latitude"], longitude=data["gps"]["longitude"]
                ),
                speed=ci.HelperSpeed(speed=data["gps"]["speed_kmph"]),
                heading=ci.HelperHeading(heading=data["gps"]["heading_deg"]),
            )
        else:
            self._gps = None

        if data.get("tw_speed_kmph") is not None or data.get("tw_dir_deg") is not None:
            self._tw_speed_kmph = data["tw_speed_kmph"]
            self._tw_dir_deg = data["tw_dir_deg"]
        else:
            self._tw_speed_kmph = None
            self._tw_dir_deg = None

        if data.get("global_path") is not None:
            self._global_path = ci.Path(
                waypoints=[
                    ci.HelperLatLon(latitude=wp["latitude"], longitude=wp["longitude"])
                    for wp in data["global_path"]["waypoints"]
                ]
            )
        else:
            self._global_path = None

        events = data.get("events") or {}
        if not isinstance(events, dict):
            raise ValueError("'events' must be a mapping of mock name to event list")
        unknown_mocks = set(events.keys()) - _VALID_MOCK_NAMES
        if unknown_mocks:
            raise ValueError(
                f"Unknown mock names in 'events': {sorted(unknown_mocks)}. "
                f"Allowed: {sorted(_VALID_MOCK_NAMES)}"
            )
        self._wind_events = _parse_wind_events(events.get("mock_wind_sensor"))
        self._gps_events = _parse_gps_events(events.get("mock_gps"))
        self._ais_events = _parse_ais_events(events.get("mock_ais"))
        self._global_path_events = _parse_global_path_events(events.get("mock_global_path"))

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

    @property
    def wind_events(self) -> tuple[WindEvent, ...]:
        return self._wind_events

    @property
    def gps_events(self) -> tuple[GpsEvent, ...]:
        return self._gps_events

    @property
    def ais_events(self) -> tuple[AisEvent, ...]:
        return self._ais_events

    @property
    def global_path_events(self) -> tuple[GlobalPathEvent, ...]:
        return self._global_path_events


def _check_fields(
    item: Any, required: frozenset, allowed: frozenset, context: str
) -> None:
    if not isinstance(item, dict):
        raise ValueError(f"{context} must be a mapping, got {type(item).__name__}")
    missing = required - item.keys()
    if missing:
        raise ValueError(f"{context} missing required fields: {sorted(missing)}")
    unknown = set(item.keys()) - allowed
    if unknown:
        raise ValueError(f"{context} has unknown fields: {sorted(unknown)}")


def _check_range(
    value: float, low: float, high: float, name: str, context: str
) -> None:
    if not (low <= value <= high):
        raise ValueError(f"{context} {name}={value} out of range [{low}, {high}]")


def _check_heading(value: float, name: str, context: str) -> None:
    # Match the runtime constraint used by mock_wind_sensor._on_set_parameters.
    if value <= -180 or value > 180:
        raise ValueError(f"{context} {name}={value} must be in (-180, 180]")


def _check_monotonic(timestamps: list[float], mock_name: str) -> None:
    for i, ts in enumerate(timestamps):
        if ts < 0:
            raise ValueError(f"{mock_name} event[{i}] timestamp={ts} must be >= 0")
        if i > 0 and ts <= timestamps[i - 1]:
            raise ValueError(
                f"{mock_name} event[{i}] timestamp={ts} must be strictly greater than "
                f"previous timestamp={timestamps[i - 1]}"
            )


def _parse_wind_events(raw: Any) -> tuple[WindEvent, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ValueError("events.mock_wind_sensor must be a list")
    events: list[WindEvent] = []
    timestamps: list[float] = []
    for i, item in enumerate(raw):
        context = f"mock_wind_sensor event[{i}]"
        _check_fields(item, _WIND_EVENT_FIELDS, _WIND_EVENT_FIELDS, context)
        ts = float(item["timestamp"])
        direction = int(item["direction_deg"])
        speed = float(item["speed_kmph"])
        _check_heading(direction, "direction_deg", context)
        _check_range(speed, 0.0, MAX_WIND_SPEED_KMPH, "speed_kmph", context)
        events.append(
            WindEvent(timestamp=ts, direction_deg=direction, speed_kmph=speed)
        )
        timestamps.append(ts)
    _check_monotonic(timestamps, "mock_wind_sensor")
    return tuple(events)


def _parse_gps_events(raw: Any) -> tuple[GpsEvent, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ValueError("events.mock_gps must be a list")
    events: list[GpsEvent] = []
    timestamps: list[float] = []
    for i, item in enumerate(raw):
        context = f"mock_gps event[{i}]"
        _check_fields(item, frozenset({"timestamp"}), _GPS_EVENT_FIELDS, context)
        ts = float(item["timestamp"])
        kwargs: dict[str, Any] = {"timestamp": ts}
        for bool_field in ("use_gps_noise", "use_ocean_drift", "use_drift_randomization"):
            if bool_field in item:
                kwargs[bool_field] = bool(item[bool_field])
        if "ocean_drift_speed_kmph" in item:
            v = float(item["ocean_drift_speed_kmph"])
            _check_range(v, 0.0, MAX_DRIFT_SPEED_KMPH, "ocean_drift_speed_kmph", context)
            kwargs["ocean_drift_speed_kmph"] = v
        if "ocean_drift_dir_deg" in item:
            v = float(item["ocean_drift_dir_deg"])
            _check_heading(v, "ocean_drift_dir_deg", context)
            kwargs["ocean_drift_dir_deg"] = v
        if "ocean_drift_accel_kmph2" in item:
            v = float(item["ocean_drift_accel_kmph2"])
            _check_range(
                v,
                -MAX_DRIFT_ACCEL_KMPH2,
                MAX_DRIFT_ACCEL_KMPH2,
                "ocean_drift_accel_kmph2",
                context,
            )
            kwargs["ocean_drift_accel_kmph2"] = v
        events.append(GpsEvent(**kwargs))
        timestamps.append(ts)
    _check_monotonic(timestamps, "mock_gps")
    return tuple(events)


def _parse_ais_events(raw: Any) -> tuple[AisEvent, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ValueError("events.mock_ais must be a list")
    events: list[AisEvent] = []
    timestamps: list[float] = []
    for i, item in enumerate(raw):
        context = f"mock_ais event[{i}]"
        _check_fields(item, _AIS_EVENT_FIELDS, _AIS_EVENT_FIELDS, context)
        ts = float(item["timestamp"])
        ships_raw = item["ships"]
        if not isinstance(ships_raw, list):
            raise ValueError(f"{context} 'ships' must be a list")
        seen_ids: set[int] = set()
        parsed_ships: list[ci.HelperAISShip] = []
        for j, ship in enumerate(ships_raw):
            ship_context = f"{context} ship[{j}]"
            _check_fields(ship, _AIS_SHIP_FIELDS, _AIS_SHIP_FIELDS, ship_context)
            ship_id = int(ship["id"])
            if ship_id <= 0:
                raise ValueError(f"{ship_context} id={ship_id} must be > 0")
            if ship_id in seen_ids:
                raise ValueError(f"{context} contains duplicate ship id {ship_id}")
            seen_ids.add(ship_id)
            _check_range(
                float(ship["lat"]), MIN_LATITUDE, MAX_LATITUDE, "lat", ship_context
            )
            _check_range(
                float(ship["lon"]), MIN_LONGITUDE, MAX_LONGITUDE, "lon", ship_context
            )
            _check_heading(float(ship["heading"]), "heading", ship_context)
            _check_range(
                float(ship["speed"]), 0.0, MAX_SHIP_SPEED_KMPH, "speed", ship_context
            )
            _check_range(
                float(ship["rot"]),
                -MAX_ROT_DEG_PER_MIN,
                MAX_ROT_DEG_PER_MIN,
                "rot",
                ship_context,
            )
            _check_range(
                float(ship["width"]), 0.0, MAX_DIMENSION_M, "width", ship_context
            )
            _check_range(
                float(ship["height"]), 0.0, MAX_DIMENSION_M, "height", ship_context
            )
            parsed_ships.append(
                ci.HelperAISShip(
                    id=ship["id"],
                    lat_lon=ci.HelperLatLon(
                        latitude=ship["lat"], longitude=ship["lon"]
                    ),
                    cog=ci.HelperHeading(heading=ship["heading"]),
                    sog=ci.HelperSpeed(speed=ship["speed"]),
                    rot=ci.HelperROT(rot=ship["rot"]),
                    width=ci.HelperDimension(dimension=ship["width"]),
                    length=ci.HelperDimension(dimension=ship["height"]),
                )
            )
        events.append(AisEvent(timestamp=ts, ships=tuple(parsed_ships)))
        timestamps.append(ts)
    _check_monotonic(timestamps, "mock_ais")
    return tuple(events)


def _parse_global_path_events(raw: Any) -> tuple[GlobalPathEvent, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ValueError("events.mock_global_path must be a list")
    events: list[GlobalPathEvent] = []
    timestamps: list[float] = []
    for i, item in enumerate(raw):
        context = f"mock_global_path event[{i}]"
        _check_fields(
            item, _GLOBAL_PATH_EVENT_FIELDS, _GLOBAL_PATH_EVENT_FIELDS, context
        )
        ts = float(item["timestamp"])
        waypoints_raw = item["waypoints"]
        if not isinstance(waypoints_raw, list) or not waypoints_raw:
            raise ValueError(f"{context} 'waypoints' must be a non-empty list")
        parsed_wps: list[ci.HelperLatLon] = []
        for j, wp in enumerate(waypoints_raw):
            wp_context = f"{context} waypoint[{j}]"
            _check_fields(wp, _WAYPOINT_FIELDS, _WAYPOINT_FIELDS, wp_context)
            lat = float(wp["latitude"])
            lon = float(wp["longitude"])
            _check_range(lat, MIN_LATITUDE, MAX_LATITUDE, "latitude", wp_context)
            _check_range(lon, MIN_LONGITUDE, MAX_LONGITUDE, "longitude", wp_context)
            parsed_wps.append(ci.HelperLatLon(latitude=lat, longitude=lon))
        events.append(GlobalPathEvent(timestamp=ts, waypoints=tuple(parsed_wps)))
        timestamps.append(ts)
    _check_monotonic(timestamps, "mock_global_path")
    return tuple(events)
