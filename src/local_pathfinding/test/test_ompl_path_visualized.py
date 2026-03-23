import matplotlib.pyplot as plt
from shapely.geometry import MultiPolygon
import custom_interfaces.msg as ci
import local_pathfinding.local_path as lp
import local_pathfinding.ompl_path as op
from rclpy.impl.rcutils_logger import RcutilsLogger


def visualize_path(
    waypoints: list[ci.HelperLatLon],
    global_path: ci.Path,
    ais_ships: ci.AISShips,
    land_multi_polygon: MultiPolygon = None,
    title: str = "Optimized Sailbot Path"
):
    """
    Visualizes the Sailbot path, global path, AIS ships, and land polygons.
    """
    fig, ax = plt.subplots(figsize=(8, 8))

    # Plot land polygons
    if land_multi_polygon:
        for polygon in land_multi_polygon.geoms:
            x, y = polygon.exterior.xy
            ax.fill(x, y, color='lightgrey', alpha=0.5, label='Land')

    # Plot AIS ships
    if ais_ships and hasattr(ais_ships, 'ships'):
        for ship in ais_ships.ships:
            ax.plot(ship.lat_lon.longitude, ship.lat_lon.latitude, 'ro', label='AIS Ship')

    # Plot global path
    if global_path and global_path.waypoints:
        lons = [wp.longitude for wp in global_path.waypoints]
        lats = [wp.latitude for wp in global_path.waypoints]
        ax.plot(lons, lats, 'b--', linewidth=1, label='Global Path')

    # Plot optimized path
    if waypoints:
        lons = [wp.longitude for wp in waypoints]
        lats = [wp.latitude for wp in waypoints]
        ax.plot(lons, lats, 'g-', linewidth=2, marker='o', label='Optimized Path')
        ax.plot(lons[0], lats[0], 'go', markersize=8, label='Start')
        ax.plot(lons[-1], lats[-1], 'gx', markersize=10, label='Goal')

    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title(title)
    ax.legend()
    ax.grid(True)
    plt.axis('equal')
    plt.show()


def generate_and_visualize_optimized_waypoints(
    gps: ci.GPS,
    ais_ships: ci.AISShips,
    global_path: ci.Path,
    target_global_waypoint: ci.HelperLatLon,
    filtered_wind_sensor: ci.WindSensor,
    planner: str,
    land_multi_polygon: MultiPolygon = None,
):
    """
    Generates the optimized path and visualizes it.
    """
    # 1. Create the path state
    path_state = lp.LocalPathState(
        gps=gps,
        ais_ships=ais_ships,
        global_path=global_path,
        target_global_waypoint=target_global_waypoint,
        filtered_wind_sensor=filtered_wind_sensor,
        planner=planner,
    )

    # 2. Create the OMPL path
    ompl_path = op.OMPLPath(
        parent_logger=RcutilsLogger(),
        local_path_state=path_state,
        land_multi_polygon=land_multi_polygon,
    )

    # 3. Get all waypoints
    waypoints_list: list[ci.HelperLatLon] = ompl_path.get_path().waypoints

    # 4. Visualize
    visualize_path(
        waypoints=waypoints_list,
        global_path=global_path,
        ais_ships=ais_ships,
        land_multi_polygon=land_multi_polygon,
        title=f"Optimized Path ({planner})"
    )

    return waypoints_list


# Example usage
testgps = ci.GPS(
    lat_lon=ci.HelperLatLon(latitude=0.0, longitude=0.8),
    heading=ci.HelperHeading(heading=45.0)
)
testais = ci.AISShips(
    ships=[
        ci.HelperAISShip(
                id=1,
                lat_lon=ci.HelperLatLon(latitude=0.3, longitude=0.3),
                cog=ci.HelperHeading(heading=-60.0),
                sog=ci.HelperSpeed(speed=20.0),
                width=ci.HelperDimension(dimension=30.0),
                length=ci.HelperDimension(dimension=200.0),
                rot=ci.HelperROT(rot=0),
            )])
testgp = ci.Path(waypoints=[
    ci.HelperLatLon(latitude=0.0, longitude=0.0),
    ci.HelperLatLon(latitude=1.0, longitude=1.0)
])
testtargetgp = ci.HelperLatLon(latitude=1.0, longitude=1.0)
testfws = ci.WindSensor(speed=ci.HelperSpeed(speed=10.0), direction=90)
testplanner = "rrtstar"

# This will both generate and visualize
generate_and_visualize_optimized_waypoints(
    testgps, testais, testgp, testtargetgp, testfws, testplanner
)
