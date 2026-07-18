from types import SimpleNamespace
from typing import Generic, TypeVar
from unittest import mock

import custom_interfaces.msg as ci
from local_pathfinding.mock_nodes.node_mock_gps import MockGPS
from local_pathfinding.mock_nodes.node_mock_wind_sensor import MockWindSensor

MessageT = TypeVar("MessageT")


class FakePublisher(Generic[MessageT]):
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.messages: list[MessageT] = []

    def publish(self, msg: MessageT) -> None:
        self.messages.append(msg)


def test_mock_gps_publishes_separate_rudder_heading() -> None:
    node = object.__new__(MockGPS)
    node._current_location = ci.HelperLatLon(latitude=49.0, longitude=-123.0)
    node._mean_speed_kmph = ci.HelperSpeed(speed=5.0)
    node._heading_deg = ci.HelperHeading(heading=-45.0)
    node._use_drift = False
    node._use_noise = False
    node._gps_pub = FakePublisher[ci.GPS]("gps")
    node._heading_pub = FakePublisher[ci.HelperHeading]("rudder")
    setattr(node, "update_speed", lambda: None)
    setattr(node, "get_next_location", lambda: None)
    logger = mock.Mock()
    setattr(node, "get_logger", lambda: logger)

    MockGPS.mock_gps_callback(node)

    assert len(node._gps_pub.messages) == 1
    assert node._gps_pub.messages[0].heading.heading == -45.0
    assert node._heading_pub.messages == [node._heading_deg]


def test_mock_wind_uses_gps_speed_and_rudder_heading_separately() -> None:
    node = object.__new__(MockWindSensor)
    node._gps_sub = SimpleNamespace(topic="gps")
    node._heading_sub = SimpleNamespace(topic="rudder")
    node._boat_speed_kmph = 1.0
    node._boat_heading_deg = 10.0
    logger = mock.Mock()
    setattr(node, "get_logger", lambda: logger)

    node.gps_callback(
        ci.GPS(
            speed=ci.HelperSpeed(speed=7.0),
            heading=ci.HelperHeading(heading=123.0),
        )
    )

    assert node._boat_speed_kmph == 7.0
    assert node._boat_heading_deg == 10.0

    node.heading_callback(ci.HelperHeading(heading=-90.0))

    assert node._boat_heading_deg == -90.0
