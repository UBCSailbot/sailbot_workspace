"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math
from typing import Optional

import numpy as np
from custom_interfaces.msg import HelperAISShip, HelperLatLon
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon, box

from local_pathfinding.coord_systems import (
    XY,
    latlon_polygon_list_to_xy_polygon_list,
    latlon_to_xy,
    meters_to_km,
)

# Constants
PROJ_HOURS_NO_COLLISION = 3  # hours
BOAT_BUFFER = 0.5  # km
COLLISION_ZONE_STRETCH_FACTOR = 1.5  # This factor changes the width of the boat collision zone


class Obstacle:
    """
    This class describes general obstacle objects which are
    anything which the sailbot must avoid.

    Do not instantiate an Obstacle object directly, use one of the subclasses instead.

    Attributes:
        reference (HelperLatLon): Lat and lon position of the next global waypoint.
        sailbot_position (HelperLatLon): Lat and lon position of SailBot.
        collision_zone ([Polygon or MultiPolygon]): Shapely geometry representing the
            obstacle's collision zone. Shape depends on the child class.
    """

    def __init__(self, reference: HelperLatLon, sailbot_position: HelperLatLon):
        self.reference = reference
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = latlon_to_xy(self.reference, self.sailbot_position_latlon)

        # Defined later by the child class
        self.collision_zone = None

    def is_valid(self, point: XY) -> bool:
        """
        Checks if a path planner's state point is inside the obstacle's collision zone.

        Args:
            point (HelperLatLon): Point representing the state point to be checked.

        Returns:
            bool: True if the point is not within the obstacle's collision zone, false otherwise.

        Raises:
            RuntimeError: If the collision zone has not yet been initialized.
        """
        if self.collision_zone is None:
            raise RuntimeError("Collision zone has not been initialized")

        return not self.collision_zone.contains(Point(*point))

    def update_collision_zone(self, **kwargs) -> None:
        """
        Updates the collision zone of the obstacle to reflect updated attributes.
        If attributes of the obstacle have not been changed, this function will have no effect.
        """
        if isinstance(self, Boat):
            # Boat Obstacle
            self._update_boat_collision_zone()

        else:
            # Land Obstacle
            self._update_land_collision_zone(  # type: ignore
                state_space=kwargs.get("state_space"),
                land_multi_polygon=kwargs.get("land_multi_polygon"),
            )

    def update_sailbot_data(
        self, sailbot_position: HelperLatLon, sailbot_speed: Optional[float] = None
    ) -> None:
        """Updates Sailbot's position, and Sailbot's speed (if the caller is a Boat object).

        Args:
            sailbot_position (HelperLatLon): Position of the SailBot.
            sailbot_speed (float): Speed of the SailBot in kmph.
        """
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = latlon_to_xy(self.reference, sailbot_position)

        if isinstance(self, Boat):
            self.sailbot_speed = sailbot_speed

    def update_reference_point(self, reference: HelperLatLon, **kwargs) -> None:
        """Updates the reference point and updates the collision zone.

        Args:
            reference (HelperLatLon): Position of the updated global waypoint.
        """
        self.reference = reference
        self.sailbot_position = latlon_to_xy(self.reference, self.sailbot_position_latlon)
        self.update_collision_zone(**kwargs)


class Land(Obstacle):
    """
    Describes land and territorial waters that which Sailbot must avoid. During runtime,
    the obstacle tracker will keep track of a single Land obstacle object and update its collision
    zone with new geometry when required.

    Attributes:
        collision_zone (MultiPolygon):
                                       A collection of Polygons in (X,Y) that define regions of
                                       land stored in the Land object. Even if the land is a single
                                       polygon, or no polygons, the collision_zone is always a
                                       MultiPolygon.
        all_land_data (MultiPolygon): MultiPolygon of absolutely all land polygons known to
                                      Sailbot.
        bbox_buffer_amount (float): The amount of square buffer around Sailbot and around the next
                                    global waypoint. In degrees lat/lon.
    """

    def __init__(
        self,
        reference: HelperLatLon,
        sailbot_position: HelperLatLon,
        all_land_data: MultiPolygon,
        bbox_buffer_amount: float,
        state_space: Polygon = None,
        land_multi_polygon: MultiPolygon = None,
    ):
        super().__init__(reference, sailbot_position)
        self.all_land_data = all_land_data
        self.bbox_buffer_amount = bbox_buffer_amount
        self._update_land_collision_zone(
            state_space=state_space, land_multi_polygon=land_multi_polygon
        )

    def _update_land_collision_zone(
        self, state_space: Polygon = None, land_multi_polygon: MultiPolygon = None
    ) -> None:
        """
        Updates the Land object's collision zone with a MultiPolygon representing
        all land obstacles within either a specified or default state space.

        Args:
            state_space (Polygon): A custom state space.
            land_multi_polygon (MultiPolygon): Custom land data. Useful for testing.
        """
        if land_multi_polygon is not None:  # for testing

            collision_zone = land_multi_polygon.buffer(0)

            if collision_zone.geom_type == "Polygon":
                collision_zone = MultiPolygon([collision_zone])

            self.collision_zone = collision_zone
            return

        if state_space is None:  # create a default one

            sailbot_box = Point(
                self.sailbot_position_latlon.longitude, self.sailbot_position_latlon.latitude
            ).buffer(self.bbox_buffer_amount, cap_style=3, join_style=2)

            waypoint_box = Point(self.reference.longitude, self.reference.latitude).buffer(
                self.bbox_buffer_amount, cap_style=3, join_style=2
            )
            state_space = box(*MultiPolygon([sailbot_box, waypoint_box]).bounds)

        latlon_polygons = self.all_land_data.intersection(state_space)

        if isinstance(latlon_polygons, MultiPolygon):
            # non-empty MultiPolygon
            xy_polygons = latlon_polygon_list_to_xy_polygon_list(
                latlon_polygons.geoms, self.reference
            )
        else:
            # single Polygon (empty or non-empty)
            # pass it inside a list for consistency
            xy_polygons = latlon_polygon_list_to_xy_polygon_list([latlon_polygons], self.reference)

        collision_zone = MultiPolygon(xy_polygons)

        if len(collision_zone.geoms) > 0:
            # if collision_zone is empty, buffer(0) will return an empty Polygon
            # buffer(0) will repair invalid/overlapping geometry
            # otherwise we cant run .contains() on it
            collision_zone = collision_zone.buffer(0)

            if collision_zone.geom_type == "Polygon":
                collision_zone = MultiPolygon([collision_zone])
        else:
            collision_zone = MultiPolygon()

        self.collision_zone = collision_zone


class Boat(Obstacle):
    """Describes boat objects which Sailbot must avoid.
    Also referred to target ships or boat obstacles.

    Attributes:
       ais_ship (HelperAISShip): AIS Ship message containing information about the boat.
       width (float): Width of the boat in km.
       length (float): Length of the boat in km.
    """

    def __init__(
        self,
        reference: HelperLatLon,
        sailbot_position: HelperLatLon,
        sailbot_speed: float,
        ais_ship: HelperAISShip,
    ):
        super().__init__(reference, sailbot_position)
        self.sailbot_speed = sailbot_speed
        self.ais_ship = ais_ship
        self.width = meters_to_km(self.ais_ship.width.dimension)
        self.length = meters_to_km(self.ais_ship.length.dimension)
        self._update_boat_collision_zone()

    def _update_boat_collision_zone(self, ais_ship: Optional[HelperAISShip] = None) -> None:
        """Sets or regenerates a Shapely Polygon that represents the boat's collision zone.

        Args:
            ais_ship (Optional[HelperAISShip]): AIS Ship message containing boat information.
        """
        if ais_ship is not None:
            if ais_ship.id != self.ais_ship.id:
                raise ValueError("Argument AIS Ship ID does not match this Boat instance's ID")

            # Ensure ais_ship instance variable is the most up to date one
            self.ais_ship = ais_ship

        # Store as local variables for performance
        ais_ship = self.ais_ship
        width = self.width
        length = self.length

        # coordinates of the center of the boat
        position = latlon_to_xy(self.reference, ais_ship.lat_lon)

        # Course over ground of the boat
        cog = ais_ship.cog.heading

        # Calculate distance the boat will travel before soonest possible collision with Sailbot
        projected_distance = self._calculate_projected_distance()

        # TODO This feels too arbitrary, maybe will incorporate ROT at a later time
        collision_zone_width = projected_distance * COLLISION_ZONE_STRETCH_FACTOR * width

        # Points of the boat collision cone polygon before rotation and centred at the origin
        boat_collision_zone = Polygon(
            [
                [-width / 2, -length / 2],
                [-collision_zone_width, length / 2 + projected_distance],
                [collision_zone_width, length / 2 + projected_distance],
                [width / 2, -length / 2],
            ]
        )

        dx, dy = position
        angle_rad = math.radians(-cog)
        sin_theta = math.sin(angle_rad)
        cos_theta = math.cos(angle_rad)

        # coefficient matrix for the 2D affine transformation of the collision zone
        transformation = np.array([cos_theta, -sin_theta, sin_theta, cos_theta, dx, dy])
        collision_zone = affine_transform(boat_collision_zone, transformation)
        self.collision_zone = collision_zone.buffer(BOAT_BUFFER, join_style=2)

    def _calculate_projected_distance(self) -> float:
        """Calculates the distance the boat obstacle will travel before collision, if
        Sailbot moves directly towards the soonest possible collision point at its current speed.
        The system is modeled by two parametric lines extending from the positions of the boat
        obstacle and sailbot respectively, in 2D space. These lines may intersect at some specific
        point and time.

        The vector that represents the Sailbot's velocity is free to point at the soonest possible
        collision point, but its magnitude is constrained.

        An in-depth explanation for this function can be found here:
        https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1881145358/Obstacle+Class+Planning

        Returns:
            float: Distance the boat will travel before collision or the max projection distance
                   if a collision is not possible.
        """
        position = latlon_to_xy(self.reference, self.ais_ship.lat_lon)

        # vector components of the boat's speed over ground
        cog_rad = math.radians(self.ais_ship.cog.heading)
        v1 = self.ais_ship.sog.speed * math.sin(cog_rad)
        v2 = self.ais_ship.sog.speed * math.cos(cog_rad)

        # coordinates of the boat
        a, b = position

        # coordinates of Sailbot
        c, d = self.sailbot_position

        quadratic_coefficients = np.array(
            [
                v1**2 + v2**2 - (self.sailbot_speed**2),  # type: ignore
                2 * (v1 * (a - c) + v2 * (b - d)),
                (a - c) ** 2 + (b - d) ** 2,
            ]
        )

        # The solution to the quadratic formula is the time until the boats collide
        quad_roots = np.roots(quadratic_coefficients)

        # filter out only positive and real roots
        quad_roots = [i for i in quad_roots if i >= 0 and i.imag == 0]  # type: ignore

        if len(quad_roots) == 0:
            # Sailbot and this Boat will never collide
            return PROJ_HOURS_NO_COLLISION * self.ais_ship.sog.speed

        # Use the smaller positive time, if there is one
        t = min(quad_roots)
        return t * self.ais_ship.sog.speed
