"""Constants used across the controller package."""

# meters, trim tab chord width is not included
CHORD_WIDTH_MAIN_SAIL = 0.23

# {m^2 / s at 10degC} and air density at 1.225 {kg / m^3}
KINEMATIC_VISCOSITY = 0.000014207

# lookup_table (List[List[Scalar]]): A list of lists or NDArray containing x-y
# data points for interpolation. Shape should be (n, 2).
# x units is reynolds number, y units is degrees
REYNOLDS_NUMBER_ALPHA_TABLE = [
    [50000, 5.75],
    [100000, 6.75],
    [200000, 7],
    [500000, 9.25],
    [1000000, 10],
]
