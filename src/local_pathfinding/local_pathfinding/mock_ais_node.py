import rclpy
from custom_interfaces.msg import AISShips, HelperAISShip
from rclpy.node import Node


"""
Defines a Mock AIS Node that publishes AIS ships to the ROS Network for testing purposes
based on following:
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

Publishers:
publisher_: Publishes mock AIS data in 'AISShips' message
"""


class MockAISNode(Node):

    def __init__(self):
        super().__init__("mock_ais_node")
        self.publisher_ = self.create_publisher(
            msg_type=AISShips, topic="ais_ships", qos_profile=10
        )
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Hard code sample AIS ship
        ship = HelperAISShip()
        ship.id = 123456789
        ship.lat_lon.latitude = 49.2827
        ship.lat_lon.longitude = -123.1207
        ship.cog.heading = 90.0
        ship.sog.speed = 10.5
        ship.rot.rot = 0
        ship.length.dimension = 100.0
        ship.width.dimension = 100.0

        msg = AISShips()
        msg.ships.append(ship)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published AISShips with {len(msg.ships)} ships")


def main(args=None):
    rclpy.init(args=args)
    node = MockAISNode()
    rclpy.spin(node)
    node.destroy_node()  # optional; otherwise will be done by gc
    rclpy.shutdown()


if __name__ == "__main__":
    main()
