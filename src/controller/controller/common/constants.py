"""Constants used across the controller package."""

# meters, trim tab chord width is not included
CHORD_WIDTH_MAIN_SAIL = 0.23

# {m^2 / s at 10degC} and air density at 1.225 {kg / m^3}
KINEMATIC_VISCOSITY = 0.000014207

# lookup_table (List[List[Scalar]]): A list of lists or NDArray containing x-y
# data points for interpolation. Shape should be (n, 2).
# x units is reynolds number, y units is degrees
# Alpha includes a 0.2 degree error
# Error is derived from moment coefficient in relation to lift coefficient of boat
REYNOLDS_NUMBER_ALPHA_TABLE = [
    [50000, 5.95],
    [100000, 6.95],
    [200000, 7.2],
    [500000, 9.45],
    [1000000, 10.2],
]
