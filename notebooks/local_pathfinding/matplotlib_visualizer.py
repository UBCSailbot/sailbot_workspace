import argparse

import cartopy.crs as ccrs
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from custom_interfaces.msg import HelperLatLon, Path
from local_pathfinding.coord_systems import latlon_to_xy


class sailbot_visualizer:
    def __init__(self, sailbot_positions, waypoints, reference_point, boat_speeds, boat_headings):
        self.sailbot_pos_lat_lon = sailbot_positions
        self.waypoints_lat_lon = waypoints
        self.reference_point_lat_lon = reference_point
        self.boat_speeds = boat_speeds
        self.boat_headings = boat_headings

        # Convert lat/lon to xy coordinates
        self.sailbot_pos_xy = [
            latlon_to_xy(reference=reference_point_lat_lon, latlon=sailbot_position_lat_lon)
            for sailbot_position_lat_lon in self.sailbot_pos_lat_lon
        ]
        self.sailbot_pos_xy = [
            latlon_to_xy(reference=reference_point_lat_lon, latlon=waypoint)
            for waypoint in local_waypoints_lat_lon.waypoints
        ]

        # Lat/Lon and XY separated
        self.waypoints_lat = [waypoint.latitude for waypoint in self.waypoints_lat_lon.waypoints]
        self.waypoints_lon = [waypoint.longitude for waypoint in self.waypoints_lat_lon.waypoints]

        self.waypoints_x = [self.sailbot_pos_xy[i].x for i in range(len(self.sailbot_pos_xy))]
        self.waypoints_y = [self.sailbot_pos_xy[i].y for i in range(len(self.sailbot_pos_xy))]

        # Boat History
        self.boat_history_lat_lon = [
            {
                "position": self.sailbot_pos_lat_lon[i],
                "waypoints_lat": self.waypoints_lat,
                "waypoints_lon": self.waypoints_lon,
                "boat_speed": self.boat_speeds[i],
                "boat_heading": self.boat_headings[i],
            }
            for i in range(len(sailbot_positions))
        ]

        self.boat_history_xy = [
            {
                "position": self.sailbot_pos_xy[i],
                "waypoints_x": self.waypoints_x,
                "waypoints_y": self.waypoints_y,
                "boat_speed": self.boat_speeds[i],
                "boat_heading": self.boat_headings[i],
            }
            for i in range(len(sailbot_positions))
        ]

    def visualize_XY(self):
        # Setup figure and axis
        fig, ax = plt.subplots()
        ax.set_title("Boat Animation")
        ax.set_xlabel("X")
        ax.set_ylabel("Y")

        # Plot waypoints
        ax.plot(
            self.waypoints_x[0], self.waypoints_y[0], "go", label="Start"
        )  # Start point (green)
        ax.plot(
            self.waypoints_x[1:-1], self.waypoints_y[1:-1], "bo", label="Intermediate"
        )  # Intermediate (blue)
        ax.plot(self.waypoints_x[-1], self.waypoints_y[-1], "ro", label="Goal")  # Goal (red)

        # Boat marker (initial position)
        (boat_marker,) = ax.plot([], [], "ro", markersize=10, label="Boat")

        # Set axis limits (you can adjust margins)
        ax.set_xlim(min(self.waypoints_x) - 10, max(self.waypoints_x) + 10)
        ax.set_ylim(min(self.waypoints_y) - 10, max(self.waypoints_y) + 10)
        ax.legend()

        # Animation update function
        def update(frame):
            boat_x = self.boat_history_xy[frame]["position"].x
            boat_y = self.boat_history_xy[frame]["position"].y
            boat_marker.set_data(boat_x, boat_y)
            return (boat_marker,)

        # Create animation
        anim = FuncAnimation(
            fig, update, frames=len(self.boat_history_xy), interval=1000, blit=True
        )

        plt.show()

    def visualize_latlon(self):
        # Setup figure and axis
        fig, ax = plt.subplots(subplot_kw={"projection": ccrs.PlateCarree()})
        ax.set_title("Boat Animation")
        ax.set_xlabel("Longitude")
        ax.set_ylabel("Latitude")
        ax.coastlines(resolution="50m")  # Add coastlines
        ax.gridlines(draw_labels=True)
        # Plot waypoints
        ax.plot(
            self.waypoints_lon[0],
            self.waypoints_lat[0],
            "go",
            label="Start",
            transform=ccrs.PlateCarree(),
        )  # Start point (green)
        ax.plot(
            self.waypoints_lon[1:-1],
            self.waypoints_lat[1:-1],
            "bo",
            label="Intermediate",
            transform=ccrs.PlateCarree(),
        )  # Intermediate (blue)
        ax.plot(
            self.waypoints_lon[-1],
            self.waypoints_lat[-1],
            "ro",
            label="Goal",
            transform=ccrs.PlateCarree(),
        )  # Goal (red)

        # Boat marker (initial position)
        (boat_marker,) = ax.plot([], [], "ro", markersize=10, label="Boat")

        ax.set_extent([
            min(self.waypoints_lon) - 1, max(self.waypoints_lon) + 1,
            min(self.waypoints_lat) - 1, max(self.waypoints_lat) + 1
        ], crs=ccrs.PlateCarree())
        ax.legend()

        # Animation update function
        def update(frame):
            boat_x = self.boat_history_lat_lon[frame]["position"].longitude
            boat_y = self.boat_history_lat_lon[frame]["position"].latitude
            boat_marker.set_data([boat_x], [boat_y])  # Use boat_marker instead of self.boat_marker
            return (boat_marker,)


        # Create animation
        anim = FuncAnimation(
            fig, update, frames=len(self.boat_history_lat_lon), interval=1000, blit=True
        )

        plt.show()


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

boat_speed = [10.0, 5, 15, 20, 9]  # Speed in knots
boat_heading = [0, 275, 280, 285, 290]  # Heading in degrees


visualizer = sailbot_visualizer(
    sailbot_positions=sailbot_position_lat_lons,
    waypoints=local_waypoints_lat_lon,
    reference_point=reference_point_lat_lon,
    boat_speeds=boat_speed,
    boat_headings=boat_heading,
)

parser = argparse.ArgumentParser()
parser.add_argument("--xy", action="store_true", help="Visualize xy coordinates")
parser.add_argument("--latlon", action="store_true", help="Visualize latlon coordinates")

args = parser.parse_args()

if args.latlon:
    # Visualize the boat's path in XY coordinates
    visualizer.visualize_latlon()
else:
    # Visualize the boat's path in lat/lon coordinates
    visualizer.visualize_XY()
    print(" Please specify --xy or --latlon to visualize the coordinates otherwise visualizing XY")
