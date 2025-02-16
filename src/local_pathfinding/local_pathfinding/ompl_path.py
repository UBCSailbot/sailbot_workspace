"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List

import pyompl
from custom_interfaces.msg import HelperLatLon
from rclpy.impl.rcutils_logger import RcutilsLogger

# Fix the import
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
from local_pathfinding.objectives import get_sailing_objective

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

# OMPL logging: only log warnings and above
# ou.setLogLevel(ou.LOG_WARN)


class OMPLPath:
    """Represents the general OMPL Path.

    Attributes
        _logger (RcutilsLogger): ROS logger of this class.
        _simple_setup (og.SimpleSetup): OMPL SimpleSetup object.
        solved (bool): True if the path is a solution to the OMPL query, else false.
    """

    def __init__(
        self,
        parent_logger: RcutilsLogger,
        max_runtime: float,
        local_path_state: LocalPathState,
    ):
        """Initialize the OMPLPath Class. Attempt to solve for a path.

        Args:
            parent_logger (RcutilsLogger): Logger of the parent class.
            max_runtime (float): Maximum amount of time in seconds to look for a solution path.
            local_path_state (LocalPathState): State of Sailbot.

        Fields not mentioned in Args:
            box_buffer (float): buffer around the sailbot position and the goal position in km
        """
        self.box_buffer = 1
        self._logger = parent_logger.get_child(name="ompl_path")

        self._simple_setup = self._init_simple_setup(local_path_state)  # this needs state

        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        # TODO: play around with simplifySolution()
        # if self.solved:
        #     # try to shorten the path
        #     simple_setup.simplifySolution()

    def get_cost(self):
        """Get the cost of the path generated.

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def get_waypoints(self) -> List[HelperLatLon]:
        """Get a list of waypoints for the boat to follow.

        Returns:
            list: A list of tuples representing the x and y coordinates of the waypoints.
                  Output an empty list and print a warning message if path not solved.
        """
        if not self.solved:
            self._logger.warning("Trying to get the waypoints of an unsolved OMPLPath")
            return []

        solution_path = self._simple_setup.getSolutionPath()

        waypoints = []

        for state in solution_path.getStates():
            waypoint_XY = cs.XY(state.getX(), state.getY())  # TODO causes SEG FAULT
            waypoint_latlon = cs.xy_to_latlon(self.state.reference_latlon, waypoint_XY)
            waypoints.append(
                HelperLatLon(
                    latitude=waypoint_latlon.latitude, longitude=waypoint_latlon.longitude
                )
            )

        return waypoints

    def create_space(self, position) -> Polygon:
        """ Create a space around the given position. Position is the center of the space and
            is a tuple of x and y.
        """
        space = Point(position[0], position[1]).buffer(self.box_buffer, cap_style=3, join_style=2)
        return space

    def update_objectives(self):
        """Update the objectives on the basis of which the path is optimized.
        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def _init_simple_setup(self, local_path_state) -> pyompl.SimpleSetup:
        self.state = local_path_state

        # Create buffered spaces and extract their centers
        state_domain = self.create_space(self.state.position)
        start_x, start_y = self.state.position  # Use original position for coordinates

        if not self.state.global_path:
            goal_polygon = self.create_space([0, 0])
            goal_x, goal_y = (0, 0)
        else:
            goal_position = self.state.global_path[-1]
            goal_polygon = self.create_space(goal_position)
            goal_x, goal_y = goal_position

        # create an SE2 state space: rotation and translation in a plane
        space = pyompl.SE2StateSpace()

        # set the bounds of the state space
        bounds = pyompl.RealVectorBounds(dim=2)
        big_polygon = box(*MultiPolygon([state_domain, goal_polygon]).bounds)
        x_min, y_min, x_max, y_max = big_polygon.bounds
        # x_min, x_max = state_domain
        # y_min, y_max = state_range

        if x_max <= x_min or y_max <= y_min:
            raise ValueError(f"Invalid bounds: x=[{x_min}, {x_max}], y=[{y_min}, {y_max}]")
        bounds.setLow(0, x_min)
        bounds.setLow(1, y_min)
        bounds.setHigh(0, x_max)
        bounds.setHigh(1, y_max)
        """self._logger.debug(
            "state space bounds: "
            f"x=[{bounds.low[0]}, {bounds.high[0]}]; "
            f"y=[{bounds.low[1]}, {bounds.high[1]}]"
        )"""
        bounds.check()  # check if bounds are valid
        space.setBounds(bounds)

        # create a simple setup object
        simple_setup = pyompl.SimpleSetup(space)
        simple_setup.setStateValidityChecker(is_state_valid)

        # set the goal and start states of the simple setup object
        start = pyompl.ScopedState(space)
        goal = pyompl.ScopedState(space)
        # start_x, start_y = start_state
        # goal_x, goal_y = goal_state
        start.setXY(start_x, start_y)
        goal.setXY(goal_x, goal_y)
        """self._logger.debug(
            "start and goal state: "
            f"start=({start().getX()}, {start().getY()}); "
            f"goal=({goal().getX()}, {goal().getY()})"
        )"""
        simple_setup.setStartAndGoalStatesSE2(start, goal)

        # Constructs a space information instance for this simple setup
        space_information = simple_setup.getSpaceInformation()

        # figure this out
        self.state.planner = pyompl.RRTstar(space_information)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here

        objective = get_sailing_objective(
            space_information,
            simple_setup,
            # This too
            self.state.heading,
            self.state.wind_direction,
            self.state.wind_speed,
        )

        simple_setup.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        simple_setup.setPlanner(pyompl.RRTstar(space_information))

        return simple_setup


def is_state_valid(state: pyompl.SE2StateSpace) -> bool:
    """Evaluate a state to determine if the configuration collides with an environment obstacle.

    Args:
        state (ob.SE2StateSpace): State to check.

    Returns:
        bool: True if state is valid, else false.
    """
    # TODO: implement obstacle avoidance here
    # note: `state` is of type `SE2StateInternal`, so we don't need to use the `()` operator.
    return state.getX() < 0.6


def get_planner_class():
    """Choose the planner to use for the OMPL query.

    Args:
        planner (str): Name of the planner to use.

    Returns:
        Tuple[str, Type[ob.Planner]]: The name and class of the planner to use for the OMPL query,
            defaults to RRT* if `planner` is not implemented in this function.
    """
    return "rrtstar", pyompl.RRTstar
