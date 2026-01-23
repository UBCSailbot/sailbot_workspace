"""Mock wind sensor node for ROS 2.

This node publishes mock data that mimics the boat's wind sensors.

It takes **true wind** inputs in the **global frame** via ROS parameters and publishes
**apparent wind** outputs in the **boat frame** on ``filtered_wind_sensor``.

Node name:
* ``mock_wind_sensor``

Publishes:
* topic: ``filtered_wind_sensor`` (custom_interfaces/WindSensor)

Subscribes:
* topic: ``gps`` (custom_interfaces/GPS) for boat heading and speed

Parameters:
* ``pub_period_sec`` (double, required): publish period (seconds)
* ``tw_speed_kmph`` (double): true wind speed (kmph). Set via ``wind_params.sh`` script only.
* ``tw_dir_deg`` (int): true wind direction in global frame (deg). Set via ``wind_params.sh``
script only.
    * Valid range: (-180, 180]

Setting True Wind Parameters:
* Must modify ``wind_params.yaml`` before running the shell script.
* Must ensure both ``mock_wind_sensor`` and ``mock_gps`` nodes are running.
* Run ``./local_pathfinding/mock_nodes/wind_params.sh`` to load parameters.
* **WARNING**: Do NOT use ``ros2 param set`` for true wind values. This causes parameter
  mismatch between nodes and breaks calculations. Always use the shell script.
"""

from typing import List

import custom_interfaces.msg as ci
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

import local_pathfinding.mock_nodes.shared_utils as sc
import local_pathfinding.wind_coord_systems as wcs


class MockWindSensor(Node):
    def __init__(self):
        super().__init__("mock_wind_sensor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("test_plan", rclpy.Parameter.Type.STRING),
                # Default true wind parameters (can be overridden via params file or CLI)
            ],
        )

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )
        self.test_plan = self.get_parameter("test_plan").get_parameter_value().string_value

        self._mock_wind_sensor_timer = self.create_timer(
            timer_period_sec=self.pub_period_sec, callback=self.mock_wind_sensor_callback
        )
        self._wind_sensors_pub = self.create_publisher(
            msg_type=ci.WindSensor,
            topic="filtered_wind_sensor",
            qos_profile=10,
        )
        self._gps_sub = self.create_subscription(
            msg_type=ci.GPS, topic="gps", callback=self.gps_callback, qos_profile=10
        )

        # Cached parameter-backed values (updated through on-set-parameters callback).
        self.initialize_mock_wind_sensor_params()

        self.add_on_set_parameters_callback(self._on_set_parameters)

    def mock_wind_sensor_callback(self):
        aw_dir_deg, aw_speed_kmph = wcs.get_apparent_wind(
            self._tw_dir_deg,
            self._tw_speed_kmph,
            self._boat_heading_deg,
            self._boat_speed_kmph,
            ret_rad=False,
        )

        aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(self._boat_heading_deg, aw_dir_deg)
        msg = ci.WindSensor(
            speed=ci.HelperSpeed(speed=aw_speed_kmph),
            direction=int(aw_dir_boat_coord_deg),
        )
        self.get_logger().debug(f"Publishing to {self._wind_sensors_pub.topic}: {msg}")
        self._wind_sensors_pub.publish(msg)

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """ROS2 parameter update callback.

        Applies updates to true wind speed/direction. Values take effect on the next publish tick.
        """
        try:
            for p in params:
                if p.name == "tw_dir_deg":
                    new_direction_deg = int(p.value)
                    sc.validate_tw_dir_deg(new_direction_deg)
                    self._tw_dir_deg = new_direction_deg
                else:
                    self._tw_speed_kmph = p.value
            return SetParametersResult(successful=True)
        except Exception:
            reason = "Please enter the direction in (-180, 180]."
            return SetParametersResult(successful=False, reason=reason)

    def gps_callback(self, msg: ci.GPS) -> None:
        """Callback function for the GPS subscription. Updates the boat's position.

        Args:
            msg (ci.GPS): The GPS message containing the boat's position.
        """

        self.get_logger().debug(f"received n {self._wind_sensors_pub.topic}: {msg}")
        self._boat_heading_deg = msg.heading.heading
        self._boat_speed_kmph = msg.speed.speed

    def initialize_mock_wind_sensor_params(self):
        """Initialize mock wind sensor parameters from test_plan file."""

        data = sc.read_test_plan_file(self.test_plan)

        wind_sensor_data = data.get("wind_sensor", {})

        if wind_sensor_data is {}:
            self.get_logger().fatal(
                f"No wind_sensor section found in test plan file {self.test_plan}. "
                "Using default parameters."
            )

        self._tw_dir_deg = int(wind_sensor_data["tw_dir_deg"])
        self._tw_speed_kmph = float(wind_sensor_data["tw_speed_kmph"])

        gps_data = data.get("gps", {})

        if gps_data is {}:
            self.get_logger().fatal(
                f"No gps section found in test plan file {self.test_plan}. "
                "Using default parameters."
            )

        state = gps_data.get("state", {})

        if state is {}:
            self.get_logger().fatal(
                f"No gps section found in test plan file {self.test_plan}. "
                "Using default parameters."
            )

        self._boat_heading_deg = float(state["heading"])
        self._boat_speed_kmph = float(state["speed"])


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
