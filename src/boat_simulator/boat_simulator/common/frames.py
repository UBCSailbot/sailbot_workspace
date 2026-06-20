from dataclasses import dataclass
from typing import Generic, TypeVar

import numpy as np
from rclpy.logging import get_logger

_LOGGER = get_logger("units_frames")

Quantity = TypeVar("Quantity")
Frame = TypeVar("Frame")


@dataclass(frozen=True)
class Vec3(Generic[Quantity, Frame]):
    data: np.ndarray
    frame: Frame

    def __post_init__(self) -> None:
        if self.data.shape != (3,):
            raise _LOGGER.warn(f"Vec3 expected shape (3,), got {self.data.shape}")


class Vec2(Generic[Quantity, Frame]):
    data: np.ndarray
    frame: Frame

    def __post_init__(self) -> None:
        if self.data.shape != (2,):
            raise _LOGGER.warn(f"Vec3 expected shape (3,), got {self.data.shape}")
