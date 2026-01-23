import math

import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node

import local_pathfinding.coord_systems as cs
import local_pathfinding.mock_nodes.shared_utils as sc

"""
Defines a Mock AIS Node that publishes AIS ships to the ROS Network for testing purposes

Publishers:
    publisher_: Publishes mock AIS data in 'AISShips' message
"""

AIS_SHIPS_FILE_PATH = (
    "/workspaces/sailbot_workspace/src/local_pathfinding/mock_ais_files/" "ais_ships.csv"
)


class MockAISNode(Node):

    def __init__(self):
        super().__init__("mock_ais_node")
        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(
            msg_type=ci.AISShips, topic="ais_ships", qos_profile=10
        )
        self.timer_period = 0.5
        self.first_run = True
        self.ships = []
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ci.AISShips()
        if self.first_run:
            # read mock ais ships from test plan file
            data = sc.read_test_plan_file(self.test_plan)

            ais_section = data.get("ais", {}) if data else {}
            vessels = ais_section.get("vessels", [])

            for row in vessels:
                # Expecting: [id, lat, lon, heading, speed, rot, width, length]
                if not isinstance(row, (list, tuple)) or len(row) < 8:
                    continue

                ship = ci.HelperAISShip()
                ship.id = int(row[0])
                ship.lat_lon.latitude = float(row[1])
                ship.lat_lon.longitude = float(row[2])
                ship.cog.heading = float(row[3])
                ship.sog.speed = float(row[4])
                ship.rot.rot = int(row[5])
                ship.width.dimension = float(row[6])
                ship.length.dimension = float(row[7])
                self.ships.append(ship)
                msg.ships.append(ship)
            self.first_run = False

        else:
            csv_ship_ids = []
            ships_to_remove = []

            data = sc.read_test_plan_file(self.test_plan)
            ais_section = data.get("ais", {}) if data else {}
            vessels = ais_section.get("vessels", [])

            for row in vessels:
                if not isinstance(row, (list, tuple)) or len(row) < 1:
                    continue
                id = int(row[0])
                csv_ship_ids.append(id)
                current_ship_ids = [ship.id for ship in self.ships]
                if id > 0 and id not in current_ship_ids and len(row) >= 8:
                    ship = ci.HelperAISShip()
                    ship.id = int(row[0])
                    ship.lat_lon.latitude = float(row[1])
                    ship.lat_lon.longitude = float(row[2])
                    ship.cog.heading = float(row[3])
                    ship.sog.speed = float(row[4])
                    ship.rot.rot = int(row[5])
                    ship.width.dimension = float(row[6])
                    ship.length.dimension = float(row[7])
                    self.ships.append(ship)
                    msg.ships.append(ship)

            for ship in self.ships:
                if ship.id not in csv_ship_ids:
                    ships_to_remove.append(ship)

            for ship in ships_to_remove:
                self.ships.remove(ship)

            for ship in self.ships:
                self.update_ship_position(ship)
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
        time = self.timer_period
        ref = ci.HelperLatLon(latitude=0.0, longitude=0.0)
        ship_latlon = ci.HelperLatLon(
            latitude=ship.lat_lon.latitude, longitude=ship.lat_lon.longitude
        )

        # Get ROT in radians per second
        rot = ship.rot.rot
        if rot == -128:
            rot_dpm = 0
        elif abs(rot) == 127:
            rot_dpm = 10
        else:
            rot_dpm = (rot / 4.733) ** 2
        rot_rps = math.radians(rot_dpm / 60)
        if rot < 0:
            rot_rps *= -1

        # Update heading
        ship.cog.heading += math.degrees(rot_rps * time)
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
        new_x = ship_cartesian.x + vx * time
        new_y = ship_cartesian.y + vy * time

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
    node.destroy_node()  # optional; otherwise will be done by gc
    rclpy.shutdown()


if __name__ == "__main__":
    main()
