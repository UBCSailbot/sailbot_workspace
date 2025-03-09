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
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.coord_systems import XY
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
        _box_buffer (float): buffer around the sailbot position and the goal position in km
        solved (bool): True if the path is a solution to the OMPL query, else false.

    Static Attributes
        obstacles (List[Polygon]): The list of all obstacles Sailbot is currently aware of.
    """

    obstacles: List[ob.Obstacle] = []

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
        self._box_buffer = 1
        self._logger = parent_logger.get_child(name="ompl_path")
        self._simple_setup = self._init_simple_setup(local_path_state)  # this needs state

        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        # TODO: play around with simplifySolution()
        # if self.solved:
        #     # try to shorten the path
        #     simple_setup.simplifySolution()

    def init_obstacles(
        self, local_path_state: LocalPathState, state_space_xy: Polygon
    ) -> List[Polygon]:
        """Extracts obstacle data from local_path_state and compiles it into a list of Polygons

        Places Boats first in the list as states are more likely to conflict with Boats than Land.

        Args:
            local_path_state (LocalPathState): a wrapper class containing all the necessary data
                                                to generate the obstacle list.

        """
        obstacles = []
        ais_ships = local_path_state.ais_ships.ships  # type:ignore

        for ship in ais_ships:
            obstacles.append(
                ob.Boat(
                    local_path_state.reference_latlon,
                    local_path_state.position,
                    local_path_state.speed,
                    ship,
                )
            )

        return obstacles

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
            waypoint_XY = cs.XY(state.getX(), state.getY())
            waypoint_latlon = cs.xy_to_latlon(self.state.reference_latlon, waypoint_XY)
            waypoints.append(
                HelperLatLon(
                    latitude=waypoint_latlon.latitude, longitude=waypoint_latlon.longitude
                )
            )

        return waypoints

    def create_buffer_around_position(self: OMPLPath, position: XY) -> Polygon:
        """Create a space around the given position. Position is the center of the space and
        is a tuple of x and y.
        """
        space = Point(position.x, position.y).buffer(self._box_buffer, cap_style=3, join_style=2)
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
        start_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, self.state.position)
        start_box = self.create_buffer_around_position(start_position_in_xy)
        start_x = start_position_in_xy.x
        start_y = start_position_in_xy.y

        if not self.state.global_path:
            goal_polygon = self.create_buffer_around_position(cs.XY(0, 0))
            goal_x, goal_y = (0.0, 0.0)
        else:
            goal_position = self.state.global_path[-1]
            goal_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, goal_position)
            goal_polygon = self.create_buffer_around_position(goal_position_in_xy)
            goal_x, goal_y = goal_position_in_xy

        # create an SE2 state space: rotation and translation in a plane
        space = pyompl.SE2StateSpace()

        # set the bounds of the state space
        bounds = pyompl.RealVectorBounds(dim=2)
        state_space = box(*MultiPolygon([start_box, goal_polygon]).bounds)
        x_min, y_min, x_max, y_max = state_space.bounds

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

        self.obstacles = self.init_obstacles(local_path_state, state_space_xy=state_space)

        # create a simple setup object
        simple_setup = pyompl.SimpleSetup(space)
        simple_setup.setStateValidityChecker(OMPLPath.is_state_valid)

        start = pyompl.ScopedState(space)
        goal = pyompl.ScopedState(space)
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
        """Evaluate a state to determine if the configuration collides with an environment
        obstacle.

        Args:
            state (ob.SE2StateSpace): State to check.

        Returns:
            bool: True if state is valid, else false.
        """
        state_is_valid = True

        for o in OMPLPath.obstacles:
            state_is_valid = o.is_valid(cs.XY(state.getX(), state.getY()))
            if not state_is_valid:
                return state_is_valid

        return state_is_valid


def get_planner_class():
    """Choose the planner to use for the OMPL query.

    Args:
        planner (str): Name of the planner to use.

    Returns:
        Tuple[str, Type[ob.Planner]]: The name and class of the planner to use for the OMPL query,
            defaults to RRT* if `planner` is not implemented in this function.
    """
    return "rrtstar", pyompl.RRTstar
