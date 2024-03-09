"""Custom types used for type hinting in the boat simulator package."""

from enum import Enum
from typing import TypeVar, Union

import numpy as np
from numpy.typing import NDArray

# Any attribute of a class that extends Enum
EnumAttr = TypeVar("EnumAttr", bound=Enum)

# Scalar value that can be an integer or a float
Scalar = Union[int, float]

# Used in cases where support for scalars or arrays of scalars are needed.
ScalarOrArray = Union[Scalar, NDArray[Union[np.int32, np.float32]]]
