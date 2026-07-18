"""Mock wind sensor node for ROS 2.

This node publishes mock data that mimics the boat's wind sensors.

It takes **true wind** inputs in the **global frame** via ROS parameters and publishes
**apparent wind** outputs in the **boat frame** on ``filtered_wind_sensor``.

Node name:
* ``mock_wind_sensor``

Publishes:
* topic: ``filtered_wind_sensor`` (custom_interfaces/WindSensor)

Subscribes:
* topic: ``gps`` (custom_interfaces/GPS) for boat speed
* topic: ``rudder`` (custom_interfaces/HelperHeading) for e-compass boat heading

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

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from test_plans.test_plan import TestPlan

import custom_interfaces.msg as ci
import local_pathfinding.wind_coord_systems as wcs


class MockWindSensor(Node):
    def __init__(self):
        super().__init__("mock_wind_sensor")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pub_period_sec", rclpy.Parameter.Type.DOUBLE),
                ("tw_speed_kmph", rclpy.Parameter.Type.DOUBLE),
                ("tw_dir_deg", rclpy.Parameter.Type.INTEGER),
                ("test_plan", rclpy.Parameter.Type.STRING),
            ],
        )

        test_plan = TestPlan(self.get_parameter("test_plan").get_parameter_value().string_value)
        if test_plan.gps is None or test_plan.heading is None:
            raise ValueError("MockWindSensor requires gps and heading_deg test-plan data")
        self._tw_dir_deg = test_plan.tw_dir_deg
        self._tw_speed_kmph = test_plan.tw_speed_kmph
        self._boat_heading_deg = test_plan.heading.heading
        self._boat_speed_kmph = test_plan.gps.speed.speed

        self.pub_period_sec = (
            self.get_parameter("pub_period_sec").get_parameter_value().double_value
        )

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
        self._heading_sub = self.create_subscription(
            msg_type=ci.HelperHeading,
            topic="rudder",
            callback=self.heading_callback,
            qos_profile=10,
        )

        # Parameter Event Handler (Parameters can change over the life of the simulation)
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def mock_wind_sensor_callback(self):
        aw_dir_deg, aw_speed_kmph = wcs.tw_gc_to_aw_gc(
            self._tw_dir_deg,
            self._tw_speed_kmph,
            self._boat_heading_deg,
            self._boat_speed_kmph,
        )

        aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(self._boat_heading_deg, aw_dir_deg)
        msg = ci.WindSensor(
            speed=ci.HelperSpeed(speed=aw_speed_kmph),
            direction=int(aw_dir_boat_coord_deg),
        )
        self.get_logger().debug(f"Publishing to {self._wind_sensors_pub.topic}: {msg}")
        self._wind_sensors_pub.publish(msg)

    def _on_set_parameters(self, params: List[Parameter]) -> SetParametersResult:
        """This callback function serves as a guard to ensure values entered with `ros2 param set`
        are valid before they are assigned to the parameters.

        Applies updates to true wind speed/direction. Values take effect on the next publish tick.

        Rejects if tw_dir_deg is not in (-180, 180].
        """
        for p in params:
            if p.name == "tw_dir_deg":
                tw_dir_deg = int(p.value)
                if tw_dir_deg <= -180 or tw_dir_deg > 180:
                    return SetParametersResult(
                        successful=False, reason="tw_dir_deg must be in (-180, 180]"
                    )
                self._tw_dir_deg = tw_dir_deg
            else:
                self._tw_speed_kmph = p.value
        return SetParametersResult(successful=True)

    def gps_callback(self, msg: ci.GPS) -> None:
        """Update boat speed from the GPS subscription.

        Args:
            msg (ci.GPS): GPS position and speed data.
        """

        self.get_logger().debug(f"Received data from {self._gps_sub.topic}: {msg}")
        self._boat_speed_kmph = msg.speed.speed

    def heading_callback(self, msg: ci.HelperHeading) -> None:
        """Update boat heading from the e-compass ``rudder`` subscription."""

        self.get_logger().debug(f"Received data from {self._heading_sub.topic}: {msg}")
        self._boat_heading_deg = msg.heading


def main(args=None):
    rclpy.init(args=args)
    mock_wind_sensor = MockWindSensor()

    rclpy.spin(node=mock_wind_sensor)

    mock_wind_sensor.destroy_node()
    rclpy.shutdown()
