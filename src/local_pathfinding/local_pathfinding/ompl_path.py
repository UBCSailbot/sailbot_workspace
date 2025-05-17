"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from __future__ import annotations

import pickle
from typing import TYPE_CHECKING, Any, List, Union

import custom_interfaces.msg as ci
import numpy as np
from ompl import base
from ompl import control as oc
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

# OMPL logging: only log warnings and above
ou.setLogLevel(ou.LOG_WARN)

BOX_BUFFER_SIZE = 1.0  # km


class OMPLPath:
    """Represents the general OMPL Path.

    Attributes
        _logger (RcutilsLogger): ROS logger of this class.
        _simple_setup (og.SimpleSetup): OMPL SimpleSetup object.
        _box_buffer (float): buffer around the sailbot position and the goal position in km
        solved (bool): True if the path is a solution to the OMPL query, else false.

    Static Attributes
        all_land_data (MultiPolygon): All land polygons along the entire global voyage
        obstacles (List[Polygon]): The list of all obstacles Sailbot is currently aware of.
                                   This is a static attribute so that OMPL can access it when
                                   accessing the is_state_valid function pointer.
    """

    all_land_data = None
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
        self._box_buffer = BOX_BUFFER_SIZE
        self._logger = parent_logger.get_child(name="ompl_path")
        self._simple_setup = self._init_simple_setup(local_path_state)
        self.solved = self._simple_setup.solve(time=max_runtime)

    @staticmethod
    def init_obstacles(
        local_path_state: LocalPathState, state_space_xy: Polygon = None
    ) -> List[ob.Obstacle]:
        """Extracts obstacle data from local_path_state and compiles it into a list of Obstacles

        Places Boats first in the list as states are more likely to conflict with Boats than Land.

        Args:
            local_path_state (LocalPathState): a wrapper class containing all the necessary data
                                                to generate the obstacle list.
            state_space_xy: (Polygon): the current state space in which we want to initialize all
                                       obstacles

        """
        OMPLPath.obstacles = []

        # BOATS
        ais_ships = local_path_state.ais_ships

        if ais_ships:
            for ship in ais_ships:
                OMPLPath.obstacles.append(
                    ob.Boat(
                        local_path_state.reference_latlon,
                        local_path_state.position,
                        local_path_state.speed,
                        ship,
                    )
                )

        # LAND
        if state_space_xy is None:
            state_space_latlon = None
        else:
            # convert the current XY state space back to latlon coordinates
            # to generate land obstacles
            state_space_latlon = cs.xy_polygon_to_latlon_polygon(
                reference=local_path_state.reference_latlon, poly=state_space_xy
            )

        if OMPLPath.all_land_data is None:
            try:
                LAND = load_pkl(
                    "/workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/land.pkl"
                )
            except RuntimeError as e:
                exit(f"could not load the land.pkl file {e}")

        OMPLPath.obstacles.append(
            ob.Land(
                reference=local_path_state.reference_latlon,
                sailbot_position=local_path_state.position,
                all_land_data=LAND,
                state_space=state_space_latlon,
            )
        )

        # obstacles are also stored in the local_path_state object
        # so that the navigate node can access and publish the obstacles
        local_path_state.obstacles = OMPLPath.obstacles
        return OMPLPath.obstacles  # for testing

    def get_cost(self):
        """Get the cost of the path generated.

        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def get_path(self) -> ci.Path:
        """Get the collection of waypoints for the boat to follow.

        Returns:
            ci.Path: A collection of lat lon coordinates for the waypoints to follow.
                    First waypoint should be the current position and final waypoint should be the
                    next global waypoint.
                    Output an empty Path and print a warning message if path not solved.
        """
        if not self.solved:
            self._logger.warning("Trying to get the waypoints of an unsolved OMPLPath")
            return ci.Path()

        solution_path = self._simple_setup.getSolutionPath()

        waypoints = []

        for state in solution_path.getStates():
            waypoint_XY = cs.XY(state.getX(), state.getY())
            waypoint_latlon = cs.xy_to_latlon(self.state.reference_latlon, waypoint_XY)
            waypoints.append(
                ci.HelperLatLon(
                    latitude=waypoint_latlon.latitude, longitude=waypoint_latlon.longitude
                )
            )

        return ci.Path(waypoints=waypoints)

    def create_buffer_around_position(self: OMPLPath, position: cs.XY) -> Polygon:
        """Create a space around the given position. Position is the center of the space and
        is a tuple of x and y.
        """
        space = Point(position.x, position.y).buffer(self._box_buffer, cap_style=3, join_style=2)
        return space

    def _init_simple_setup(self, local_path_state) -> oc.SimpleSetup:
        self.state = local_path_state

        # Create buffered spaces and extract their centers
        start_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, self.state.position)
        start_box = self.create_buffer_around_position(start_position_in_xy)
        start_x = start_position_in_xy.x
        start_y = start_position_in_xy.y

        goal_position = self.state.global_path.waypoints[-1]
        goal_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, goal_position)
        goal_polygon = self.create_buffer_around_position(goal_position_in_xy)
        goal_x, goal_y = goal_position_in_xy

        # create an SE2 state space: rotation and translation in a plane
        space = base.SE2StateSpace()

        # set the bounds of the state space
        bounds = base.RealVectorBounds(dim=2)
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

        OMPLPath.init_obstacles(local_path_state=local_path_state, state_space_xy=state_space)

        cspace = oc.RealVectorControlSpace(space, 2)
        cbounds = base.RealVectorBounds(2)
        cbounds.setLow(0, 0.0)
        cbounds.setLow(1, -1.0)
        cbounds.setHigh(0, 5.0)
        cbounds.setHigh(1, 1.0)
        cspace.setBounds(cbounds)

        ss = oc.SimpleSetup(cspace)
        si = ss.getSpaceInformation()
        ss.setStateValidityChecker(base.StateValidityCheckerFn(OMPLPath.is_state_valid))

        def ode(state, control, duration, out):
            x = state.getX()
            y = state.getY()
            theta = state.getYaw()
            v = control[0]
            omega = control[1]
            out[0] = v * np.cos(theta)
            out[1] = v * np.sin(theta)
            out[2] = omega

        def propagate(start, control, duration, result):
            x = start.getX()
            y = start.getY()
            theta = start.getYaw()
            v = control[0]
            omega = control[1]
            dt = duration
            result.setX(x + v * np.cos(theta) * dt)
            result.setY(y + v * np.sin(theta) * dt)
            result.setYaw(theta + omega * dt)

        ss.setStatePropagator(oc.StatePropagatorFn(propagate))

        start = base.State(space)
        goal = base.State(space)

        start[0] = start_x
        start[1] = start_y
        start[2] = 0.0  # yaw in radians, adjust if needed

        goal[0] = goal_x
        goal[1] = goal_y
        goal[2] = 0.0  # yaw in radians, adjust if needed

        ss.setStartAndGoalStates(start, goal, threshold=10.0)

        planner = oc.RRT(si)
        ss.setPlanner(planner)

        return ss

    def is_state_valid(state: Union[base.State, base.SE2StateInternal]) -> bool:
        """Evaluate a state to determine if the configuration collides with an environment
        obstacle.

        Args:
            state (base.SE2StateInternal): State to check.

        Returns:
            bool: True if state is valid, else false.
        """

        for o in OMPLPath.obstacles:

            if isinstance(state, base.State):  # for testing purposes
                state_is_valid = o.is_valid(cs.XY(state().getX(), state().getY()))

            else:  # when OMPL uses this function, it will pass in an SE2StateInternal object
                state_is_valid = o.is_valid(cs.XY(state.getX(), state.getY()))

            if not state_is_valid:
                # uncomment this if you want to log which states are being labeled invalid
                # its commented out for now to avoid unnecessary file I/O

                # if isinstance(state, base.State):  # only happens in unit tests
                #     log_invalid_state(state=cs.XY(state().getX(), state().getY()), obstacle=o)
                # else:  # happens in prod
                #     log_invalid_state(state=cs.XY(state.getX(), state.getY()), obstacle=o)
                return False

        return True


def log_invalid_state(state: cs.XY, obstacle: ob.Obstacle):
    """
    Logs details about a state and the obstacle that makes it invalid for use in a path.
    """
    with open(
        "/workspaces/sailbot_workspace/src/local_pathfinding/local_pathfinding/invalid_states.log",
        "a",
    ) as log_file:
        log_file.write(
            f"State at ({state.x:.2f},{state.y:.2f}) was invalidated by obstacle: {type(obstacle)}\n"  # noqa
        )


def get_planner_class():
    """Choose the planner to use for the OMPL query.

    Args:
        planner (str): Name of the planner to use.

    Returns:
        Tuple[str, Type[base.Planner]]: The name and class of the planner to use for the OMPL
        query, defaults to RRT* if `planner` is not implemented in this function.
    """
    return "rrtstar", og.RRTstar


def load_pkl(file_path: str) -> Any:
    with open(file_path, "rb") as f:
        return pickle.load(f)
