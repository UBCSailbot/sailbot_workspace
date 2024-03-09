from typing import List

import numpy as np
from numpy.typing import NDArray
from scipy import interpolate

from controller.common.types import Scalar


class LUT:
    """
    Class for performing look-up table interpolation.

    Methods:
        __init__: Initializes the LUT object with lookup table data and interpolation method.
        __call__: Calls the interpolation method with the given input.
    """

    def __init__(
        self,
        lookup_table: List[List[Scalar]] | NDArray,
        interpolation_method: str = "linear",
    ):
        """
        Initializes the LUT object.

        Args:
            lookup_table (List[List[Scalar]] or NDArray): A list of lists or NDArray containing x-y
            data points for interpolation. Shape should be (n, 2)
            interpolation_method (str): Interpolation method to use. Default is "linear".

        Raises:
            ValueError: If the specified interpolation method is unknown
            or if the table shape is incorrect.
        """
        if isinstance(lookup_table, np.ndarray):
            table = lookup_table
        else:
            table = np.array(lookup_table)

        self.__verifyTable(table)
        self.x = table[:, 0]
        self.y = table[:, 1]

        self.__interpolation_method = interpolation_method

        match self.__interpolation_method:
            case "linear":
                self.__interpolation_function = self.__linearInterpolation
            case "spline":
                self.__interpolation_function = self.__splineInterpolation
            case _:
                raise ValueError(
                    self.__interpolation_method + " is an unknown interpolation method!"
                )

    def __call__(self, x: float) -> float:
        """
        Calls the interpolation method with the given input.

        Args:
            x (float): The input value to interpolate.

        Returns:
            float: The interpolated value using the interpolation method defined when LUT instance
            creation.
        """
        return self.__interpolation_function(x)

    def __linearInterpolation(self, x: Scalar) -> float:
        output = np.interp(x, self.x, self.y)
        if isinstance(output, np.ndarray):
            raise ValueError(
                "linear interpolation returned a NDArray when it should have returned a float"
            )
        return output

    def __splineInterpolation(self, x: Scalar) -> float:
        cs = interpolate.CubicSpline(self.x, self.y)
        return cs(x)

    def __verifyTable(self, table: NDArray) -> None:
        if (len(table.shape) != 2) or table.shape[1] != 2:
            raise ValueError(f"Input table has shape {table.shape}, but expected shape of (n, 2)")
