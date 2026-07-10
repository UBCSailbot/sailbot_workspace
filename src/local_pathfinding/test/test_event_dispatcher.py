from dataclasses import dataclass

from local_pathfinding.mock_nodes.event_dispatcher import EventDispatcher


@dataclass(frozen=True)
class _FakeEvent:
    timestamp: float
    value: int


def test_empty_dispatcher_yields_nothing():
    dispatcher: EventDispatcher = EventDispatcher([])
    assert list(dispatcher.pop_fired(100.0)) == []
    assert dispatcher.remaining == 0


def test_events_fire_when_elapsed_reaches_timestamp():
    e1 = _FakeEvent(timestamp=1.0, value=1)
    e2 = _FakeEvent(timestamp=3.0, value=2)
    dispatcher = EventDispatcher([e1, e2])
    assert list(dispatcher.pop_fired(0.5)) == []
    assert list(dispatcher.pop_fired(1.0)) == [e1]
    assert list(dispatcher.pop_fired(2.9)) == []
    assert list(dispatcher.pop_fired(3.0)) == [e2]


def test_events_fire_only_once():
    e = _FakeEvent(timestamp=1.0, value=1)
    dispatcher = EventDispatcher([e])
    assert list(dispatcher.pop_fired(5.0)) == [e]
    assert list(dispatcher.pop_fired(10.0)) == []


def test_multiple_events_fire_in_one_call():
    events = [
        _FakeEvent(timestamp=1.0, value=1),
        _FakeEvent(timestamp=2.0, value=2),
        _FakeEvent(timestamp=3.0, value=3),
    ]
    dispatcher = EventDispatcher(events)
    assert list(dispatcher.pop_fired(5.0)) == events


def test_t0_event_fires_at_zero_elapsed():
    e = _FakeEvent(timestamp=0.0, value=1)
    dispatcher = EventDispatcher([e])
    assert list(dispatcher.pop_fired(0.0)) == [e]


def test_remaining_tracks_unfired_events():
    e1 = _FakeEvent(timestamp=1.0, value=1)
    e2 = _FakeEvent(timestamp=2.0, value=2)
    dispatcher = EventDispatcher([e1, e2])
    assert dispatcher.remaining == 2
    list(dispatcher.pop_fired(1.5))
    assert dispatcher.remaining == 1
    list(dispatcher.pop_fired(3.0))
    assert dispatcher.remaining == 0


def test_events_yielded_in_original_order():
    events = [
        _FakeEvent(timestamp=0.0, value=1),
        _FakeEvent(timestamp=0.0, value=2),
    ]
    dispatcher = EventDispatcher(events)
    yielded = list(dispatcher.pop_fired(0.0))
    assert [e.value for e in yielded] == [1, 2]
