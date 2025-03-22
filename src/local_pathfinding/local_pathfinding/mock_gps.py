"""
Mock class for the GPS. Publishes basic GPS data to the ROS network.
"""
import custom_interfaces.msg as ci
import rclpy
from rclpy.node import Node
from geopy.distance import great_circle
from local_pathfinding.coord_systems import degrees_to_radians


class MockGPS(Node):

    def __init__(self):
        """Initialize the MockGPS class. The class is used to publish mock mock gps
        data to the ROS network.

        Args:
            Node (Node): The ROS node that the class will run on.

        Attributes:
            __mock_gps_timer (Timer): Timer to call the mock gps callback function.
            __mock_gps_publisher (Publisher): Publisher for the gps data.
            __mean_speed (HelperSpeed): Constant speed of the boat.
            __current_location (HelperLatLon): Current location of the boat.
            __mean_heading (HelperHeading): Constant heading of the boat.
        """
        super().__init__("mock_gps")

        # Declare ROS parameters (qos depth and publish period)
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

        # Mock GPS timere
        self.__mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

        # Mock GPS publisher initialization
        self.__gps_pub = self.create_publisher(
            msg_type=ci.GPS,
            topic="gps",
            qos_profile=10,
        )

        # Desired heading subscriber
        self.__desired_heading_sub = self.create_subscription(
            msg_type=ci.DesiredHeading,
            topic="desired_heading",
            callback=self.desired_heading_callback,
            qos_profile=10
        )

        self.__mean_speed = ci.HelperSpeed(speed=15.0)  # mean boat speed in kmph
        self.__current_location = ci.HelperLatLon(latitude=49.29, longitude=-126.32)
        self.__heading = ci.HelperHeading(heading=-60.0)  # in degrees, heading of the boat

    def mock_gps_callback(self) -> None:
        """Callback function for the mock GPS timer. Publishes mock gps data to the ROS
        network.
        """
        self.get_next_location()
        msg: ci.GPS = ci.GPS(lat_lon=self.__current_location,
                             speed=self.__mean_speed, heading=self.__heading)
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, heading: {msg.heading}")
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, speed: {msg.speed}")
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, latitude: {msg.lat_lon.latitude}"
        )
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, longitude: {msg.lat_lon.longitude}"
        )
        self.__gps_pub.publish(msg)

    def get_next_location(self) -> None:
        """Get the next location by following the great circle. Assumes constant speed and heading
        """
        # distance travelled = speed * calback time (s)
        distance_km: float = self.__mean_speed.speed * (self.pub_period_sec/3600)
        start: tuple[float, float] = (self.__current_location.latitude,
                                      self.__current_location.longitude)
        radian_angle = degrees_to_radians(self.__heading.heading)
        destination = great_circle(kilometers=distance_km).destination(start, radian_angle)
        self.__current_location = ci.HelperLatLon(latitude=destination.latitude,
                                                  longitude=destination.longitude)

    def desired_heading_callback(self, msg: ci.DesiredHeading):
        """Callback for topic desired heading

        Args:
            msg (ci.DesiredHeading): The desired heading for the boat. 
        """
        self._logger.debug(f"Received data from {self.__desired_heading_sub.topic}: {msg}")
        self.__heading = msg.heading


def main(args=None):
    rclpy.init(args=args)
    mock_gps = MockGPS()

    rclpy.spin(node=mock_gps)

    mock_gps.destroy_node()
    rclpy.shutdown()
