from shapely.geometry import box, MultiPolygon, Point
from typing import Any
from custom_interfaces.msg import HelperLatLon, Path
from local_pathfinding.coord_systems import latlon_to_xy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


sailbot_position_lat_lons = [
    HelperLatLon(latitude=49.50000, longitude=-139.922429),
    HelperLatLon(latitude=49.667979, longitude=-140.807750),
    HelperLatLon(latitude=49.979489, longitude=-141.808892),
    HelperLatLon(latitude=50.331437, longitude=-142.951804),
    HelperLatLon(latitude=50.822548, longitude=-144.336486),
]
local_waypoints_lat_lon = Path(
    waypoints=[
        HelperLatLon(latitude=49.208985, longitude=-139.922429),
        HelperLatLon(latitude=49.567979, longitude=-140.807750),
        HelperLatLon(latitude=49.979489, longitude=-141.708892),
        HelperLatLon(latitude=50.331437, longitude=-142.851804),
        HelperLatLon(latitude=50.722548, longitude=-144.236486),
    ]
)
reference_point_lat_lon = HelperLatLon(
    latitude=50.722548, longitude=-144.236486
)  # Last global waypoint

# Convert lat/lon to xy coordinates
sailbot_position_xy = [
    latlon_to_xy(reference=reference_point_lat_lon, latlon=sailbot_position_lat_lon)
    for sailbot_position_lat_lon in sailbot_position_lat_lons
]
local_waypoints_xy = [
    latlon_to_xy(reference=reference_point_lat_lon, latlon=waypoint)
    for waypoint in local_waypoints_lat_lon.waypoints
]


waypoints_lat = [waypoint.latitude for waypoint in local_waypoints_lat_lon.waypoints]
waypoints_lon = [waypoint.longitude for waypoint in local_waypoints_lat_lon.waypoints]

waypoints_x = [local_waypoints_xy[i].x for i in range(len(local_waypoints_xy))]
waypoints_y = [local_waypoints_xy[i].y for i in range(len(local_waypoints_xy))]

boat_speed = [10.0, 5, 15, 20, 9]  # Speed in knots
boat_heading = [0, 275, 280, 285, 290]  # Heading in degrees

boat_history_lat_lon = [
    {
        "position": sailbot_position_lat_lons[i],
        "waypoints_lat": waypoints_lat,
        "waypoints_lon": waypoints_lon,
        "boat_speed": boat_speed[i],
        "boat_heading": boat_heading[i],
    }
    for i in range(5)
]

boat_history_xy = [
    {
        "position": sailbot_position_xy[i],
        "waypoints_x": waypoints_x,
        "waypoints_y": waypoints_y,
        "boat_speed": boat_speed[i],
        "boat_heading": boat_heading[i],
    }
    for i in range(5)
]

print("Boat History (lat/lon):", boat_history_lat_lon)
print("Boat History (xy):", boat_history_xy)

current_position = 0


# Setup figure and axis
fig, ax = plt.subplots()
ax.set_title("Boat Animation")
ax.set_xlabel("X")
ax.set_ylabel("Y")

# Plot waypoints
ax.plot(waypoints_x[0], waypoints_y[0], "go", label="Start")  # Start point (green)
ax.plot(waypoints_x[1:-1], waypoints_y[1:-1], "bo", label="Intermediate")  # Intermediate (blue)
ax.plot(waypoints_x[-1], waypoints_y[-1], "ro", label="Goal")  # Goal (red)

# Boat marker (initial position)
(boat_marker,) = ax.plot([], [], "ro", markersize=10, label="Boat")

# Set axis limits (you can adjust margins)
ax.set_xlim(min(waypoints_x) - 10, max(waypoints_x) + 10)
ax.set_ylim(min(waypoints_y) - 10, max(waypoints_y) + 10)
ax.legend()


# Animation update function
def update(frame):
    boat_x = boat_history_xy[frame]["position"].x
    boat_y = boat_history_xy[frame]["position"].y
    boat_marker.set_data(boat_x, boat_y)
    return (boat_marker,)


# Create animation
anim = FuncAnimation(fig, update, frames=len(boat_history_xy), interval=1000, blit=True)

plt.show()
