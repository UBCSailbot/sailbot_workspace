import pytest
import rclpy
from rclpy.node import Node
import custom_interfaces as ci


class TestMockWindSensorNode(Node):

    def __init__(self):
        super().__init__(node_name="test_mock_wind_sensor")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("mode", rclpy.Parameter.Type.STRING),
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("path_planner", rclpy.Parameter.Type.STRING),
                ("use_mock_land", False),
            ],
        )

        # subscribers
        self.gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )
        self.filtered_wind_sensor_sub = self.create_subscription(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            callback=self.filtered_wind_sensor_callback,
            qos_profile=10,
        )


class FakeGPSNode(Node):
    def __init__(self, boat_heading, boat_speed):
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

        # Mock GPS timer
        self.__mock_gps_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_gps_callback
        )

        # Mock GPS publisher initialization
        self.__gps_pub = self.create_publisher(
            msg_type=ci.GPS,
            topic="gps",
            qos_profile=10,
        )
        self._lat_lon = ci.HelperLatlon(latitude=0.0, longitude=0.0)
        self._boat_heading = boat_heading
        self._boat_speed = boat_speed

    def mock_gps_callback(self) -> None:
        """Callback function for the mock GPS timer. Publishes mock gps data to the ROS
        network.
        """
        msg = ci.GPS(
            lat_lon=self._lat_lon, speed=self._boat_speed, heading=self._boat_heading
        )
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, heading: {msg.heading}")
        self.get_logger().debug(f"Publishing to {self.__gps_pub.topic}, speed: {msg.speed}")
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, latitude: {msg.lat_lon.latitude}"
        )
        self.get_logger().debug(
            f"Publishing to {self.__gps_pub.topic}, longitude: {msg.lat_lon.longitude}"
        )
        self.__gps_pub.publish(msg)
