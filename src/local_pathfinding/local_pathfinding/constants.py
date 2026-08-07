"""Constants shared by local pathfinding nodes and data consumers."""

import custom_interfaces.msg as ci

HEADING_UNAVAILABLE = 360.0
GPS_UNAVAILABLE = ci.GPS(
    lat_lon=ci.HelperLatLon(latitude=91.0, longitude=181.0),
    speed=ci.HelperSpeed(speed=-1.0),
    heading=ci.HelperHeading(heading=HEADING_UNAVAILABLE),
)
WIND_SENSOR_UNAVAILABLE = ci.WindSensor(
    speed=ci.HelperSpeed(speed=-1.0),
    direction=0,
)
AIS_SHIPS_UNAVAILABLE = ci.AISShips(
    ships=[ci.HelperAISShip(id=0)],
)
DESIRED_HEADING_UNAVAILABLE = ci.DesiredHeading(
    heading=ci.HelperHeading(heading=HEADING_UNAVAILABLE),
    sail=False,
)
