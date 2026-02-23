"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math
from abc import abstractmethod
from typing import Optional

import custom_interfaces.msg as ci
import numpy as np
from shapely import prepared
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon

import local_pathfinding.coord_systems as cs

# Constants
PREDICTION_HORIZON = 10.0  # seconds
DT = 0.5
PROJ_DISTANCE_NO_COLLISION = 0.0
BOAT_BUFFER = 0.25  # km
COLLISION_ZONE_STRETCH_FACTOR = 1.25  # This factor changes the width of the boat collision zone


class Obstacle:
    """
    This class describes general obstacle objects which are
    anything which the sailbot must avoid.

    Do not instantiate an Obstacle object directly, use one of the subclasses instead.

    Attributes:
        reference (ci.HelperLatLon): Lat and lon position of the next global waypoint.
        sailbot_position (ci.HelperLatLon): Lat and lon position of SailBot.
        collision_zone ([Polygon or MultiPolygon]): Shapely geometry representing the
            obstacle's collision zone. Shape depends on the child class.
    """

    def __init__(
        self,
        reference: ci.HelperLatLon,
        sailbot_position: ci.HelperLatLon,
        collision_zone: MultiPolygon = None,
    ):
        self.reference = reference
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = cs.latlon_to_xy(self.reference, self.sailbot_position_latlon)

        # Defined later by the child class
        self.collision_zone = collision_zone

    def is_valid(self, point: cs.XY) -> bool:
        """
        Checks if a path planner's state point is inside the obstacle's collision zone.

        Args:
            point (ci.HelperLatLon): Point representing the state point to be checked.

        Returns:
            bool: True if the point is not within the obstacle's collision zone, false otherwise.

        Raises:
            RuntimeError: If the collision zone has not yet been initialized.
        """
        if self.collision_zone is None:
            raise RuntimeError("Collision zone has not been initialized")

        return not self.collision_zone.contains(Point(*point))

    @abstractmethod
    def update_collision_zone(self, **kwargs) -> None:
        """
        Updates the collision zone of the obstacle to reflect updated attributes.
        """
        pass

    def update_sailbot_data(
        self, sailbot_position: ci.HelperLatLon, sailbot_speed: Optional[float] = None
    ) -> None:
        """Updates Sailbot's position, and Sailbot's speed (if the caller is a Boat object).

        Args:
            sailbot_position (ci.HelperLatLon): Position of the SailBot.
            sailbot_speed (float): Speed of the SailBot in kmph.
        """
        self.sailbot_position_latlon = sailbot_position
        self.sailbot_position = cs.latlon_to_xy(self.reference, sailbot_position)

    def update_reference_point(self, reference: ci.HelperLatLon, **kwargs) -> None:
        """Updates the reference point and updates the collision zone.

        Args:
            reference (ci.HelperLatLon): Position of the updated global waypoint.
        """
        self.reference = reference
        self.sailbot_position = cs.latlon_to_xy(self.reference, self.sailbot_position_latlon)
        self.update_collision_zone(**kwargs)

    def print_info(self):
        for name, value in self.__dict__.items():
            print(name, value)


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
        reference: ci.HelperLatLon,
        sailbot_position: ci.HelperLatLon,
        all_land_data: MultiPolygon,
        bbox_buffer_amount: float = 0.1,
        state_space_latlon: Polygon = None,
        land_multi_polygon: MultiPolygon = None,
    ):
        super().__init__(reference, sailbot_position)
        self.all_land_data = all_land_data
        self.bbox_buffer_amount = bbox_buffer_amount
        self.update_collision_zone(
            state_space_latlon=state_space_latlon, land_multi_polygon=land_multi_polygon
        )

    def update_collision_zone(self, **kwargs) -> None:
        """
        Updates the Land object's collision zone with a MultiPolygon representing
        all land obstacles within either a specified or default state space.

        Args:
            state_space_latlon (Polygon): A custom state space.
            land_multi_polygon (MultiPolygon): Custom land data. Useful for testing.
        """
        land_multi_polygon = kwargs.get("land_multi_polygon", None)
        state_space_latlon = kwargs.get("state_space_latlon", None)
        if land_multi_polygon is not None:  # for testing (injecting mock land data)
            self.collision_zone = MultiPolygon(
                cs.latlon_polygon_list_to_xy_polygon_list(land_multi_polygon.geoms, self.reference)
            )
            prepared.prep(self.collision_zone)
            return

        if state_space_latlon is None:
            raise ValueError("state_space_latlon must not be None")

        latlon_polygons = self.all_land_data.intersection(state_space_latlon)

        if isinstance(latlon_polygons, MultiPolygon):
            # non-empty MultiPolygon
            xy_polygons = cs.latlon_polygon_list_to_xy_polygon_list(
                latlon_polygons.geoms, self.reference
            )
        else:
            # single Polygon (empty or non-empty)
            # pass it inside a list for consistency
            xy_polygons = cs.latlon_polygon_list_to_xy_polygon_list(
                [latlon_polygons], self.reference
            )

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
        prepared.prep(collision_zone)
        self.collision_zone = collision_zone


class Boat(Obstacle):
    """Describes boat objects which Sailbot must avoid. Also referred to target ships or boat
    obstacles.

    Attributes:
       ais_ship (ci.HelperAISShip): AIS Ship message containing information about the boat.
       width (float): Width of the boat in km.
       length (float): Length of the boat in km.
    """

    def __init__(
        self,
        reference: ci.HelperLatLon,
        sailbot_position: ci.HelperLatLon,
        sailbot_speed: float,
        ais_ship: ci.HelperAISShip,
    ):
        super().__init__(reference, sailbot_position)
        self.sailbot_speed = sailbot_speed
        self.ais_ship = ais_ship
        self.width = cs.meters_to_km(self.ais_ship.width.dimension)
        self.length = cs.meters_to_km(self.ais_ship.length.dimension)
        self._raw_collision_zone = None
        self.update_collision_zone()

    def update_sailbot_data(
        self, sailbot_position: ci.HelperLatLon, sailbot_speed: Optional[float] = None
    ) -> None:
        """Updates Sailbot's position, and Sailbot's speed (if provided).

        Args:
            sailbot_position (ci.HelperLatLon): Position of the SailBot.
            sailbot_speed (float): Speed of the SailBot in kmph.
        """
        super().update_sailbot_data(sailbot_position, sailbot_speed)
        if sailbot_speed is not None:
            self.sailbot_speed = sailbot_speed

    def update_collision_zone(self, **kwargs) -> None:
        """Sets or regenerates a Shapely Polygon that represents the boat's collision zone.

        Args:

            ais_ship (Optional[ci.HelperAISShip]): AIS Ship message containing information about
                                                   the target ship.
        """
        ais_ship = kwargs.get("ais_ship", None)
        if ais_ship is not None:
            if ais_ship.id != self.ais_ship.id:
                raise ValueError("Argument AIS Ship ID does not match this Boat instance's ID")
            self.ais_ship = ais_ship

        # Calculate ROT in radians per second
        rot = self.ais_ship.rot.rot
        rot_rps = cs.rot_to_rad_per_sec(rot)

        # Calculate current and future heading and projected distances
        dt = DT
        speed_kmps = self.ais_ship.sog.speed / 3600.0
        cog_rad = math.radians(self.ais_ship.cog.heading)
        x, y = cs.latlon_to_xy(self.reference, self.ais_ship.lat_lon)
        projected_distance = self._calculate_projected_distance(cog_rad)
        bow_y = self.length / 2

        if projected_distance == PROJ_DISTANCE_NO_COLLISION:
            # If no collision detected, create small collision zone around the boat
            RADIUS_MULTIPLIER = 5
            radius = cs.km_to_meters(max(self.width, self.length)) * RADIUS_MULTIPLIER
            boat_collision_zone = Point(-self.width / 2, -self.length / 2).buffer(
                radius, resolution=16
            )
            raw = self._translate_collision_zone(boat_collision_zone)
            self._raw_collision_zone = raw
            self.collision_zone = raw.buffer(radius + BOAT_BUFFER, join_style=2)
            prepared.prep(self.collision_zone)

        elif abs(rot_rps) < 1e-4:
            # Straight line
            collision_zone_width = projected_distance * COLLISION_ZONE_STRETCH_FACTOR * self.width
            boat_collision_zone = Polygon(
                [
                    [-self.width / 2, -self.length / 2],
                    [-collision_zone_width, self.length / 2 + projected_distance],
                    [collision_zone_width, self.length / 2 + projected_distance],
                    [self.width / 2, -self.length / 2],
                ]
            )
            raw = self._translate_collision_zone(boat_collision_zone)
            self._raw_collision_zone = raw
            self.collision_zone = raw.buffer(BOAT_BUFFER, join_style=2)
            prepared.prep(self.collision_zone)
        else:
            # Turning motion
            R = speed_kmps / abs(rot_rps)
            turn_angle = rot_rps * dt

            # Center of rotation in world frame
            cx = x - R * math.sin(cog_rad) * math.copysign(1, rot_rps)
            cy = y + R * math.cos(cog_rad) * math.copysign(1, rot_rps)

            # Rotate the ship around the center
            future_x = cx + R * math.sin(cog_rad + turn_angle)
            future_y = cy + R * math.cos(cog_rad + turn_angle)

            delta_heading = rot_rps * dt
            future_cog_rad = cog_rad + delta_heading
            future_projected_distance = self._calculate_projected_distance(
                future_cog_rad, position_override=(future_x, future_y)
            )

            A = [0.0, bow_y]
            B = [0.0, bow_y + projected_distance]
            C = [
                future_projected_distance * math.sin(delta_heading),
                bow_y + future_projected_distance * math.cos(delta_heading),
            ]

            boat_collision_zone = Polygon([A, B, C])
            raw = self._translate_collision_zone(boat_collision_zone)
            self._raw_collision_zone = raw
            self.collision_zone = raw.buffer(BOAT_BUFFER, join_style=2)
            prepared.prep(self.collision_zone)

    def _translate_collision_zone(self, boat_collision_zone):
        # this code block translates and rotates the collision zone to the correct position
        dx, dy = cs.latlon_to_xy(self.reference, self.ais_ship.lat_lon)
        angle_rad = math.radians(-self.ais_ship.cog.heading)
        sin_theta = math.sin(angle_rad)
        cos_theta = math.cos(angle_rad)
        transformation = np.array([cos_theta, -sin_theta, sin_theta, cos_theta, dx, dy])
        collision_zone = affine_transform(boat_collision_zone, transformation)
        return collision_zone

    def _calculate_projected_distance(self, cog_rad, position_override=None) -> float:
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
            float: Distance in km that the boat will travel before collision or the constant value
            PROJ_DISTANCE_NO_COLLISION if a collision is not possible.
        """
        if position_override is None:
            position = cs.latlon_to_xy(self.reference, self.ais_ship.lat_lon)
        else:
            position = position_override
        v1 = self.ais_ship.sog.speed * math.sin(cog_rad)
        v2 = self.ais_ship.sog.speed * math.cos(cog_rad)
        a, b = position
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
            return PROJ_DISTANCE_NO_COLLISION

        # Use the smaller positive time, if there is one
        t = min(quad_roots)
        return t * self.ais_ship.sog.speed
