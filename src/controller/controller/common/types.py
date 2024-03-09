"""Custom types used for type hinting in the controller package."""

from typing import Union

import numpy as np
from numpy.typing import NDArray

# Scalar value that can be an integer or a float
Scalar = Union[int, float]

# Used in cases where support for scalars or arrays of scalars are needed.
ScalarOrArray = Union[Scalar, NDArray[Union[np.int32, np.float32]]]
