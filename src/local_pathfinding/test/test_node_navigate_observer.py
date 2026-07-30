from collections import deque
from typing import cast
from unittest import mock

import custom_interfaces.msg as ci
import local_pathfinding.node_navigate_observer as observer
from local_pathfinding.constants import GPS_UNAVAILABLE


class FakeQueue:
    def __init__(self) -> None:
        self.items: list[object] = []

    def qsize(self) -> int:
        return len(self.items)

    def put(self, item) -> None:
        self.items.append(item)


def make_observer_shell(heading: float = 45.0) -> observer.SailbotObserver:
    node = object.__new__(observer.SailbotObserver)
    node.msg = ci.LPathData(heading=ci.HelperHeading(heading=heading))
    node.msgs = deque([node.msg])
    setattr(node, "queue", FakeQueue())
    node.last_replan_reason = ""
    logger = mock.Mock()
    setattr(node, "get_logger", lambda: logger)
    return node


def test_observer_passes_message_to_visualizer_without_separate_heading(monkeypatch) -> None:
    node = make_observer_shell()
    monkeypatch.setattr(observer.vz, "create_visualizer_state", lambda **kwargs: kwargs)

    node.update_queue()

    queue = cast(FakeQueue, node.queue)
    queued_state = cast(dict, queue.items[0])
    assert queued_state["msgs"][-1] is node.msg
    assert "heading" not in queued_state


def test_observer_warns_when_message_heading_is_unavailable(monkeypatch) -> None:
    node = make_observer_shell(heading=observer.vz.HEADING_UNAVAILABLE)
    monkeypatch.setattr(observer.vz, "create_visualizer_state", lambda **kwargs: kwargs)

    node.update_queue()

    node.get_logger().warn.assert_called_once()


def test_observer_queues_none_when_gps_is_unavailable() -> None:
    node = make_observer_shell()
    node.msg.gps = GPS_UNAVAILABLE
    node.msgs = deque([node.msg])

    node.update_queue()

    queue = cast(FakeQueue, node.queue)
    assert queue.items == [None]
