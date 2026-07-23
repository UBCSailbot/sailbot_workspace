"""Timer-driven event dispatcher for test-plan mock nodes.

Given an event list already sorted by timestamp ascending, yields events whose
timestamps have been reached. Each event is yielded exactly once across the
dispatcher's lifetime. `test_plan.py` guarantees sort order and monotonicity at
load time, so this class does not re-check them.
"""

from collections.abc import Iterable, Iterator
from typing import Any


class EventDispatcher:

    def __init__(self, events: Iterable[Any]) -> None:
        self._events: tuple[Any, ...] = tuple(events)
        self._cursor: int = 0

    def pop_fired(self, elapsed_sec: float) -> Iterator[Any]:
        """Yield every event whose timestamp is <= elapsed_sec and hasn't fired yet."""
        while (
            self._cursor < len(self._events)
            and self._events[self._cursor].timestamp <= elapsed_sec
        ):
            yield self._events[self._cursor]
            self._cursor += 1

    @property
    def remaining(self) -> int:
        return len(self._events) - self._cursor
