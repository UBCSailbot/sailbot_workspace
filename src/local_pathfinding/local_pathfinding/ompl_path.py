"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, List

import sailompl
from custom_interfaces.msg import HelperLatLon
from rclpy.impl.rcutils_logger import RcutilsLogger

import local_pathfinding.coord_systems as cs
from local_pathfinding.objectives import get_sailing_objective

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

# OMPL logging: only log warnings and above
# ou.setLogLevel(ou.LOG_WARN)


class OMPLPathState:
    def __init__(self, local_path_state: LocalPathState, logger: RcutilsLogger):
        # TODO: derive OMPLPathState attributes from local_path_state
        self.heading_direction = 45.0
        self.wind_direction = 10.0
        self.wind_speed = 1.0

        self.state_domain = (-1, 1)
        self.state_range = (-1, 1)
        self.start_state = (0.5, 0.4)
        self.goal_state = (0.5, -0.4)

        self.reference_latlon = (
            local_path_state.global_path[-1]
            if local_path_state.global_path and len(local_path_state.global_path) > 0
            else HelperLatLon(latitude=0.0, longitude=0.0)
        )

        if local_path_state:
            self.planner = None
            self.planner = pypathfinder.RRTstar()


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
        """
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

    def update_objectives(self):
        """Update the objectives on the basis of which the path is optimized.
        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def _init_simple_setup(self, local_path_state) -> pyompl.SimpleSetup:
        """Initialize and configure the OMPL SimpleSetup object.

        Returns:
            og.SimpleSetup: Encapsulates the various objects necessary to solve a geometric or
                control query in OMPL.
        """
        self.state = OMPLPathState(local_path_state, self._logger)

        # create an SE2 state space: rotation and translation in a plane
        space = pyompl.SE2StateSpace()

        # set the bounds of the state space
        bounds = pyompl.RealVectorBounds(dim=2)
        x_min, x_max = self.state.state_domain
        y_min, y_max = self.state.state_range
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
        start_x, start_y = self.state.start_state
        goal_x, goal_y = self.state.goal_state
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

        self.state.planner = pyompl.RRTstar(space_information)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here

        objective = get_sailing_objective(
            space_information,
            simple_setup,
            self.state.heading_direction,
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
