import csv
import math

import rclpy
from custom_interfaces.msg import AISShips, HelperAISShip, HelperLatLon
from rclpy.node import Node

from local_pathfinding.coord_systems import XY, latlon_to_xy, xy_to_latlon

"""
Defines a Mock AIS Node that publishes AIS ships to the ROS Network for testing purposes
based on following:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

Publishers:
publisher_: Publishes mock AIS data in 'AISShips' message
"""

AIS_SHIPS_FILE_PATH = (
    "/workspaces/sailbot_workspace/src/local_pathfinding/mock_ais_files/" "ais_ships.csv"
)


class MockAISNode(Node):

    def __init__(self):
        super().__init__("mock_ais_node")
        self.publisher_ = self.create_publisher(
            msg_type=AISShips, topic="ais_ships", qos_profile=10
        )
        self.timer_period = 0.5
        self.first_run = True
        self.ships = []
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = AISShips()
        if self.first_run:
            # read mock ais ships from csv
            with open(AIS_SHIPS_FILE_PATH, "r") as file:
                reader = csv.reader(file)
                # skip header
                next(reader)
                for row in reader:
                    ship = HelperAISShip()
                    ship.id = int(row[0])
                    ship.lat_lon.latitude = float(row[1])
                    ship.lat_lon.longitude = float(row[2])
                    ship.cog.heading = float(row[3])
                    ship.sog.speed = float(row[4])
                    ship.rot.rot = int(row[5])
                    ship.length.dimension = float(row[6])
                    ship.width.dimension = float(row[7])
                    self.ships.append(ship)
                    msg.ships.append(ship)
            self.first_run = False

        else:
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

        # TODO: update ship based on turnspeed as well

        # Set heading, speed and time
        heading = math.radians(ship.cog.heading)
        speed = ship.sog.speed / 3600
        time = self.timer_period
        ref = HelperLatLon(latitude=0.0, longitude=0.0)
        ship_latlon = HelperLatLon(
            latitude=ship.lat_lon.latitude, longitude=ship.lat_lon.longitude
        )

        # rot = ship.rot.rot
        # # Convert ROT to degrees per minute
        # if rot == -128:
        #     rot_dpm = 0
        # elif abs(rot) == 127:
        #     rot_dpm = 10
        # else:
        #     rot_dpm = (abs(rot)/4.733) ** 2

        # Convert ship position to cartesian coordinates
        ship_cartesian = latlon_to_xy(ref, ship_latlon)
        vx = speed * math.sin(heading)
        vy = speed * math.cos(heading)

        # Calculate new position
        new_x = ship_cartesian.x + vx * time
        new_y = ship_cartesian.y + vy * time

        # Convert back to lat_lon
        new_xy = XY(x=new_x, y=new_y)
        new_latlon = xy_to_latlon(ref, new_xy)

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
