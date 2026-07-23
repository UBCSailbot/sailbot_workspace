from collections import deque
from types import SimpleNamespace
from typing import cast
from unittest import mock

import custom_interfaces.msg as ci
import local_pathfinding.node_navigate_observer as observer


class FakeQueue:
    def __init__(self) -> None:
        self.items: list[object] = []

    def qsize(self) -> int:
        return len(self.items)

    def put(self, item) -> None:
        self.items.append(item)


def make_observer_shell() -> observer.SailbotObserver:
    node = object.__new__(observer.SailbotObserver)
    node.heading_sub = SimpleNamespace(topic="rudder")
    node.heading = None
    node.msg = ci.LPathData()
    node.msgs = deque([node.msg])
    setattr(node, "queue", FakeQueue())
    node.last_replan_reason = ""
    logger = mock.Mock()
    setattr(node, "get_logger", lambda: logger)
    return node


def test_observer_passes_rudder_heading_to_visualizer(monkeypatch) -> None:
    node = make_observer_shell()
    heading = ci.HelperHeading(heading=45.0)
    node.heading_callback(heading)
    monkeypatch.setattr(observer.vz, "VisualizerState", lambda **kwargs: kwargs)

    node.update_queue()

    queue = cast(FakeQueue, node.queue)
    queued_state = cast(dict, queue.items[0])
    assert queued_state["heading"] is heading


def test_observer_rejects_invalid_rudder_heading() -> None:
    node = make_observer_shell()
    previous_heading = ci.HelperHeading(heading=10.0)
    node.heading = previous_heading

    node.heading_callback(ci.HelperHeading(heading=-180.0))

    assert node.heading is previous_heading
    node.get_logger().warning.assert_called_once()
