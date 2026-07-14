import math
import time

import rclpy
from rclpy.node import Node
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
import local_pathfinding.coord_systems as cs
from local_pathfinding.mock_nodes.dispatch_event import EventDispatcher

"""
Defines a Mock AIS Node that publishes AIS ships to the ROS Network for testing purposes

Publishers:
    publisher_: Publishes mock AIS data in 'AISShips' message
"""


class MockAISNode(Node):

    def __init__(self):
        super().__init__("mock_ais_node")
        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("on_water_mock_ais", rclpy.Parameter.Type.BOOL),
                ("test_plan", rclpy.Parameter.Type.STRING),
                ("on_water_test_plan", rclpy.Parameter.Type.STRING),
            ],
        )
        on_water_test = (
            self.get_parameter("on_water_mock_ais").get_parameter_value().bool_value
        )  # to avoid unused parameter warning

        if on_water_test:
            self.get_logger().info(
                "Mock AIS is running in in on-water testing. Mock AIS data will be published "
                + "instead of real AIS data."
            )
            self.test_plan = (
                self.get_parameter("on_water_test_plan").get_parameter_value().string_value
            )
        else:
            self.get_logger().info(
                "Mock AIS is running in development mode. Mock AIS data will be published for "
                + "testing purposes."
            )
            self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(
            msg_type=ci.AISShips, topic="ais_ships", qos_profile=10
        )
        self.timer_period = 0.5

        test_plan = TestPlan(self.test_plan)
        self.ships = list(test_plan.ais)

        # Anchor for event timing: each event's `timestamp` is seconds from here.
        self._start_monotonic_sec = time.monotonic()

        # Holds the sorted AIS event list and yields each one when its timestamp is reached.
        self._ais_dispatcher = EventDispatcher(test_plan.ais_events)

        # Skip physics for one tick after an event fires so we publish the
        # event's exact coords (initial YAML ship list counts as an event too).
        self._just_snapshotted = True

        # Fire any t=0 events so the first published message reflects them.
        self._consume_events(elapsed_sec=0.0)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def _consume_events(self, elapsed_sec: float) -> bool:
        """Apply any AIS snapshot events fired by elapsed_sec. Returns True if any fired."""
        fired = False
        for event in self._ais_dispatcher.pop_fired(elapsed_sec):
            self.ships = list(event.ships)
            fired = True
        if fired:
            self._just_snapshotted = True
        return fired

    def timer_callback(self):
        msg = ci.AISShips()
        self._consume_events(time.monotonic() - self._start_monotonic_sec)
        if self._just_snapshotted:
            self._just_snapshotted = False
        else:
            for ship in self.ships:
                self.update_ship_position(ship)
        for ship in self.ships:
            msg.ships.append(ship)

        self.publisher_.publish(msg)
        for ship in msg.ships:
            self.get_logger().debug(
                f"id={ship.id}\n"
                f"latitude={ship.lat_lon.latitude}\n"
                f"longitude={ship.lat_lon.longitude}\n"
                f"heading={ship.cog.heading}\n"
                f"speed={ship.sog.speed}\n"
            )

    def update_ship_position(self, ship):
        """
        Update the ship's position based on its speed, heading and rate of turn (ROT)
        """

        # Get speed and time
        speed = ship.sog.speed / 3600
        dt = self.timer_period
        ref = ci.HelperLatLon(latitude=0.0, longitude=0.0)
        ship_latlon = ci.HelperLatLon(
            latitude=ship.lat_lon.latitude, longitude=ship.lat_lon.longitude
        )

        # Get ROT in radians per second
        rot = ship.rot.rot
        rot_rps = cs.rot_to_rad_per_sec(rot)

        # Update heading
        ship.cog.heading += math.degrees(rot_rps * dt)
        if ship.cog.heading > 180:
            ship.cog.heading -= 360
        elif ship.cog.heading <= -180:
            ship.cog.heading += 360
        heading = math.radians(ship.cog.heading)

        # Convert ship position to cartesian coordinates
        ship_cartesian = cs.latlon_to_xy(ref, ship_latlon)
        vx = speed * math.sin(heading)
        vy = speed * math.cos(heading)

        # Calculate new position
        new_x = ship_cartesian.x + vx * dt
        new_y = ship_cartesian.y + vy * dt

        # Convert back to lat_lon
        new_xy = cs.XY(x=new_x, y=new_y)
        new_latlon = cs.xy_to_latlon(ref, new_xy)

        # Update ship position
        ship.lat_lon.latitude = new_latlon.latitude
        ship.lat_lon.longitude = new_latlon.longitude


def main(args=None):
    rclpy.init(args=args)
    node = MockAISNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
