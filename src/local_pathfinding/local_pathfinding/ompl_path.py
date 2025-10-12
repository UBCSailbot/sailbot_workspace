"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from __future__ import annotations

import os
import pickle
from typing import TYPE_CHECKING, Any, Union

import custom_interfaces.msg as ci
from ompl import base
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Polygon, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_objectives import get_sailing_objective, create_buffer_around_position

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

# OMPL logging: only log warnings and above
ou.setLogLevel(ou.LOG_WARN)


LAND_KEY = -1
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
LAND_PKL_FILE_PATH = os.path.join(CURRENT_DIR, "..", "land", "pkl", "land.pkl")


class OMPLPath:
    """Represents the general OMPL Path.

    Attributes
        _logger (RcutilsLogger): ROS logger of this class.
        _simple_setup (og.SimpleSetup): OMPL SimpleSetup object.
        _box_buffer (float): buffer around the sailbot position and the goal position in km
        solved (bool): True if the path is a solution to the OMPL query, else false.

    Static Attributes
        all_land_data (MultiPolygon): All land polygons along the entire global voyage
        obstacles (Dictionary[int, Polygon]):
                                    The dictionary of all obstacles Sailbot is currently aware of.
                                    This is a static attribute so that OMPL can access it when
                                    accessing the is_state_valid function pointer.
    """

    all_land_data = None
    obstacles: Union[dict[int, ob.Obstacle], None] = None

    def __init__(
        self,
        parent_logger: RcutilsLogger,
        max_runtime: float,
        local_path_state: LocalPathState,
        land_multi_polygon: MultiPolygon = None,
    ):
        """Initialize the OMPLPath Class. Attempt to solve for a path.

        Args:
            parent_logger (RcutilsLogger): Logger of the parent class.
            max_runtime (float): Maximum amount of time in seconds to look for a solution path.
            local_path_state (LocalPathState): State of Sailbot.
        """
        self._logger = parent_logger.get_child(name="ompl_path")
        # this needs state
        self._simple_setup = self._init_simple_setup(local_path_state, land_multi_polygon)

        self.solved = self._simple_setup.solve(time=max_runtime)  # time is in seconds

        # TODO: play around with simplifySolution()
        # if self.solved:
        #     # try to shorten the path
        #     simple_setup.simplifySolution()

    @staticmethod
    def init_obstacles(
        local_path_state: LocalPathState,
        state_space_xy: Polygon = None,
        land_multi_polygon: MultiPolygon = None,
    ) -> dict[int, ob.Obstacle]:
        """Extracts obstacle data from local_path_state and compiles it into a list of Obstacles

        Places Boats first in the list as states are more likely to conflict with Boats than Land.

        Args:
            local_path_state (LocalPathState): a wrapper class containing all the necessary data
                                                to generate the obstacle list.
            state_space_xy: (Polygon): the current state space in which we want to initialize all
                                       obstacles
            land_multi_polygon: (MultiPolygon): a collection of mock land obstacles represented
                                                as polygons.

        """

        if OMPLPath.obstacles is None:
            OMPLPath.obstacles = {}

        # BOATS
        ais_ships = local_path_state.ais_ships
        current_ship_ids = [ship.id for ship in ais_ships]

        # Remove boats no longer in ais_ships
        boat_ids_to_remove = [
            ship_id
            for ship_id in OMPLPath.obstacles.keys()
            if ship_id != LAND_KEY and ship_id not in current_ship_ids
        ]
        for ship_id in boat_ids_to_remove:
            del OMPLPath.obstacles[ship_id]

        for ship in ais_ships:
            if ship.id in OMPLPath.obstacles:
                # If the ship is already in the obstacles, update its information
                existing_boat = OMPLPath.obstacles[ship.id]
                existing_boat.update_sailbot_data(
                    sailbot_position=local_path_state.position,
                    sailbot_speed=local_path_state.speed,
                )
                existing_boat.update_collision_zone()
            else:
                # Otherwise, create a new Obstacle object
                new_boat = ob.Boat(
                    local_path_state.reference_latlon,
                    local_path_state.position,
                    local_path_state.speed,
                    ship,
                )
                OMPLPath.obstacles[ship.id] = new_boat

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
                OMPLPath.all_land_data = load_pkl(LAND_PKL_FILE_PATH)
            except FileNotFoundError as e:
                exit(f"could not load the land.pkl file {e}")

        OMPLPath.obstacles[LAND_KEY] = ob.Land(
            reference=local_path_state.reference_latlon,
            sailbot_position=local_path_state.position,
            all_land_data=OMPLPath.all_land_data,
            state_space_latlon=state_space_latlon,
            land_multi_polygon=land_multi_polygon,
        )

        # obstacles are also stored in the local_path_state object
        # so that the navigate node can access and publish the obstacles
        obstacles_list = list(OMPLPath.obstacles.values())
        local_path_state.obstacles = obstacles_list
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

    def update_objectives(self):
        """Update the objectives on the basis of which the path is optimized.
        Raises:
            NotImplementedError: Method or function hasn't been implemented yet.
        """
        raise NotImplementedError

    def _init_simple_setup(self, local_path_state, land_multi_polygon) -> og.SimpleSetup:
        self.state = local_path_state

        # Create buffered spaces and extract their centers
        start_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, self.state.position)
        start_box = create_buffer_around_position(start_position_in_xy)
        start_x = start_position_in_xy.x
        start_y = start_position_in_xy.y

        goal_position_in_xy = cs.XY(0, 0)  # Global waypoint is used as the reference point
        goal_polygon = create_buffer_around_position(goal_position_in_xy)
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

        OMPLPath.init_obstacles(
            local_path_state=local_path_state,
            state_space_xy=state_space,
            land_multi_polygon=land_multi_polygon,
        )

        # create a simple setup object
        simple_setup = og.SimpleSetup(space)
        simple_setup.setStateValidityChecker(base.StateValidityCheckerFn(OMPLPath.is_state_valid))

        start = base.State(space)
        goal = base.State(space)
        start().setXY(start_x, start_y)
        goal().setXY(goal_x, goal_y)
        self._logger.debug(
            "start and goal state: "
            f"start=({start().getX()}, {start().getY()}); "
            f"goal=({goal().getX()}, {goal().getY()})"
        )
        simple_setup.setStartAndGoalStates(start, goal)

        # Constructs a space information instance for this simple setup
        space_information = simple_setup.getSpaceInformation()

        # figure this out
        self.state.planner = og.RRTstar(space_information)

        # set the optimization objective of the simple setup object
        # TODO: implement and add optimization objective here

        objective = get_sailing_objective(
            space_information,
            simple_setup,
            # This too
            self.state.heading,
            self.state.speed,
            self.state.wind_direction,
            self.state.wind_speed,
        )

        simple_setup.setOptimizationObjective(objective)

        # set the planner of the simple setup object
        planner = og.RRTstar(space_information)
        planner.setRange(200.0)
        simple_setup.setPlanner(planner)
        # print(planner)

        return simple_setup

    def is_state_valid(state: Union[base.State, base.SE2StateInternal]) -> bool:
        """Evaluate a state to determine if the configuration collides with an environment
        obstacle.

        Args:
            state (base.SE2StateInternal): State to check.

        Returns:
            bool: True if state is valid, else false.
        """
        if OMPLPath.obstacles:

            for o in OMPLPath.obstacles.values():
                if isinstance(state, base.State):  # for testing purposes
                    state_is_valid = o.is_valid(cs.XY(state().getX(), state().getY()))

                else:  # when OMPL uses this function, it will pass in an SE2StateInternal object
                    state_is_valid = o.is_valid(cs.XY(state.getX(), state.getY()))

                if not state_is_valid:
                    # uncomment this if you want to log which states are being labeled invalid
                    # its commented out for now to avoid unnecessary file I/O

                    if isinstance(state, base.State):  # only happens in unit tests
                        log_invalid_state(state=cs.XY(state().getX(), state().getY()), obstacle=o)
                    else:  # happens in prod
                        log_invalid_state(state=cs.XY(state.getX(), state.getY()), obstacle=o)
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
