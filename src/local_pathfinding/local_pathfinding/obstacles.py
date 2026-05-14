"""Describes obstacles which the Sailbot must avoid: Boats and Land"""

import math
from abc import abstractmethod
from typing import Optional

import numpy as np
from shapely import prepared
from shapely.affinity import affine_transform
from shapely.geometry import MultiPolygon, Point, Polygon

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs

# Constants
DT = 10
PROJ_DISTANCE_NO_COLLISION = 0.0
BOAT_BUFFER = 0.1  # km
COLLISION_ZONE_STRETCH_FACTOR = 1.25  # This factor changes the width of the boat collision zone
RADIUS_MULTIPLIER = 5
STRAIGHT_LINE_ROT_THRESHOLD = 1e-4


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
        self.width_km = cs.meters_to_km(self.ais_ship.width.dimension)
        self.length_km = cs.meters_to_km(self.ais_ship.length.dimension)
        self._raw_collision_zone = Point(0, 0).buffer(0)
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
        """
        Sets or regenerates a Shapely Polygon that represents the boat's collision zone.

        The collision zone is computed in the boat's local frame then rotated and translated into
        the world frame to match the boat's current position and COG.

        Args:
            ais_ship (ci.HelperAISShip): Updated AIS ship data. Must have the same ID as this Boat.
        """

        ais_ship = kwargs.get("ais_ship", None)
        if ais_ship is not None:
            if ais_ship.id != self.ais_ship.id:
                raise ValueError("Argument AIS Ship ID does not match this Boat instance's ID")
            self.ais_ship = ais_ship

        try:
            rot_rps = cs.rot_to_rad_per_sec(self.ais_ship.rot.rot)
        except ValueError:
            rot_rps = 0.0  # invalid ROT: treat as no rotation information

        speed_kmps = self.ais_ship.sog.speed / 3600.0
        # COG is measured in degrees clockwise from North (0° = North, 90° = East)
        cog_rad = math.radians(self.ais_ship.cog.heading)

        x, y = cs.latlon_to_xy(self.reference, self.ais_ship.lat_lon)
        projected_distance = self._calculate_projected_distance(cog_rad)

        if projected_distance == PROJ_DISTANCE_NO_COLLISION:
            radius_km = max(self.width_km, self.length_km)
            self._set_circular_zone(radius_km)
        elif abs(rot_rps) < STRAIGHT_LINE_ROT_THRESHOLD:
            self._set_straight_line_zone(projected_distance)
        else:
            self._set_turning_zone(rot_rps, speed_kmps, cog_rad, x, y, projected_distance)

    def _set_circular_zone(self, radius_km: float) -> None:
        """Creates a circular collision zone centered on the boat.

        Used when no collision is projected (Case 1) or when the turning boat will no longer
        reach Sailbot (Case 3 fallback). The final zone is the circle buffered by BOAT_BUFFER:
        the circle sets the zone size, and BOAT_BUFFER adds a fixed physical clearance margin.
        """
        boat_collision_zone = Point(-self.width_km / 2, -self.length_km / 2).buffer(
            radius_km, resolution=16
        )
        self._raw_collision_zone = self._translate_collision_zone(boat_collision_zone)
        self.collision_zone = self._raw_collision_zone.buffer(
            radius_km + BOAT_BUFFER, join_style=2
        )
        prepared.prep(self.collision_zone)

    def _set_straight_line_zone(self, projected_distance: float) -> None:
        """Creates a rectangular collision zone ahead of the boat for straight-line motion.

        The rectangle already covers the full projected collision path, so only BOAT_BUFFER
        is applied. No additional radius term is needed.
        """
        boat_collision_zone = Polygon(
            [
                [-self.width_km / 2, -self.length_km / 2],
                [-self.width_km / 2, self.length_km / 2 + projected_distance],
                [self.width_km / 2, self.length_km / 2 + projected_distance],
                [self.width_km / 2, -self.length_km / 2],
            ]
        )
        self._raw_collision_zone = self._translate_collision_zone(boat_collision_zone)
        self.collision_zone = self._raw_collision_zone.buffer(BOAT_BUFFER, join_style=2)
        prepared.prep(self.collision_zone)

    def _set_turning_zone(
        self,
        rot_rps: float,
        speed_kmps: float,
        cog_rad: float,
        x: float,
        y: float,
        projected_distance: float,
    ) -> None:
        """Creates a triangular collision zone for a turning boat.

        Projects where the boat will be after DT seconds along its turning arc, then
        builds a triangle from: the bow tip, the straight-ahead collision point, and
        the collision point projected along the boat's actual turning direction.
        BOAT_BUFFER is applied to account for positional drift as the boat rotates
        around the center of curvature.
        """
        turn_radius_km = speed_kmps / abs(rot_rps)
        # Positive turn_angle = right turn (clockwise), negative = left turn (counter-clockwise)
        turn_angle = rot_rps * DT

        # The center of curvature lies 90° to the right of the heading for right turns,
        # and 90° to the left for left turns.
        # In our XY frame (x = East, y = North) with COG in degrees clockwise from North:
        #   forward direction  = (sin(cog_rad), cos(cog_rad))
        #   right-perpendicular = (cos(cog_rad), -sin(cog_rad))
        cx = x + turn_radius_km * math.cos(cog_rad) * math.copysign(1, rot_rps)
        cy = y - turn_radius_km * math.sin(cog_rad) * math.copysign(1, rot_rps)

        # Project the boat's position after DT seconds by rotating it around the center of
        # curvature.
        # A right turn (positive turn_angle) is clockwise in our coordinate system
        # (x=East, y=North).
        # Clockwise 2D rotation by angle θ:
        #   new_x = cx + (x-cx)*cos(θ) + (y-cy)*sin(θ)
        #   new_y = cy - (x-cx)*sin(θ) + (y-cy)*cos(θ)
        dx = x - cx
        dy = y - cy
        future_x = cx + dx * math.cos(turn_angle) + dy * math.sin(turn_angle)
        future_y = cy - dx * math.sin(turn_angle) + dy * math.cos(turn_angle)

        future_cog_rad = cog_rad + turn_angle
        future_projected_distance = self._calculate_projected_distance(
            future_cog_rad, position_override=(future_x, future_y)
        )

        if future_projected_distance == PROJ_DISTANCE_NO_COLLISION:
            # After turning, the boat will not reach Sailbot. Use a larger conservative zone
            # than Case 1 because the boat was on a collision course before it turned.
            radius_km = max(self.width_km, self.length_km)
            self._set_circular_zone(radius_km)
            return

        # Build a triangle in the boat's local frame (y-axis = forward, origin = boat center):
        #   A = bow tip
        #   B = collision point if the boat continued straight ahead
        #   C = collision point projected along the boat's actual turning direction
        A = [0.0, self.length_km / 2]
        B = [0.0, self.length_km / 2 + projected_distance]
        C = [
            future_projected_distance * math.sin(turn_angle),
            self.length_km / 2 + future_projected_distance * math.cos(turn_angle),
        ]

        boat_collision_zone = Polygon([A, B, C])
        self._raw_collision_zone = self._translate_collision_zone(boat_collision_zone)
        # self.collision_zone = self._raw_collision_zone.buffer(BOAT_BUFFER, join_style=2)
        self.collision_zone = self._raw_collision_zone
        prepared.prep(self.collision_zone)

    def _translate_collision_zone(self, boat_collision_zone):
        # this code block translates and rotates the collision zone to the world frame
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

        Args:
            cog_rad (float): The boat's course-over-ground in radians at the position being
                evaluated. When called for the boat's future position after turning, this should
                be the future COG, not the current one.
            position_override (tuple[float, float] | None): An (x, y) position in the XY world
                frame to use instead of the boat's current lat/lon. Used by _set_turning_zone to
                evaluate the collision distance from the boat's projected future position after it
                has turned, rather than from its current position.

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
