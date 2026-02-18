"""The local_pathfinding<->OMPL interface, represented by the OMPLPath class.

OMPL is written in C++, but Python bindings were generated to interface with OMPL in Python.
VS Code currently can't read these bindings, so LSP features (autocomplete, go to definition, etc.
won't work). The C++ API is documented on the OMPL website:
https://ompl.kavrakilab.org/api_overview.html.
"""

from __future__ import annotations

import math
import os
import pickle
from typing import TYPE_CHECKING, Any, Union

import custom_interfaces.msg as ci
from ompl import base
from ompl import geometric as og
from ompl import util as ou
from rclpy.impl.rcutils_logger import RcutilsLogger
from shapely.geometry import MultiPolygon, Point, Polygon, box

import local_pathfinding.coord_systems as cs
import local_pathfinding.obstacles as ob
from local_pathfinding.ompl_objectives import get_sailing_objective

if TYPE_CHECKING:
    from local_pathfinding.local_path import LocalPathState

ou.setLogLevel(ou.LOG_WARN)

BOX_BUFFER_SIZE_KM = 1.0
# for now this is statically defined and subject to change or may be made dynamic
MIN_TURNING_RADIUS_KM = 0.05  # 50 m
# sets an upper limit on the allowable edge length in the graph formed by the RRT* algorithm
# We set this as small as possible to reduce instances where both endpoints of a path segment are
# valid but the segment straddles an obstacle collision zone
# setting this any smaller can lead to OMPL not being able to construct a tree that reaches
# the goal state
MAX_EDGE_LEN_KM = 5.0
MAX_SOLVER_RUN_TIME_SEC = 1.0

LAND_KEY = -1
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
LAND_PKL_FILE_PATH = os.path.join(CURRENT_DIR, "..", "land", "pkl", "land.pkl")
DISTANCE_THRESHOLD = 1e-9


class OMPLPath:
    """Represents the general OMPL Path.

    Attributes
        _logger (RcutilsLogger): ROS logger of this class.
        _simple_setup (og.SimpleSetup): OMPL SimpleSetup object.
        _box_buffer (float): buffer around the sailbot position and the goal position in km
        ** solved (bool): True if the path is a solution to the OMPL query, else false. **
        ** do null checks on solved. True == object exists and False == None **
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
        local_path_state: LocalPathState,
        land_multi_polygon: MultiPolygon = None,
    ):
        """Initialize the OMPLPath Class. Attempt to solve for a path.

        Args:
            parent_logger (RcutilsLogger): Logger of the parent class.
            local_path_state (LocalPathState): State of Sailbot.
        """
        self._box_buffer = BOX_BUFFER_SIZE_KM

        self._logger = parent_logger.get_child(name="ompl_path")
        # Store the state, which includes the reference point used for this path
        self.state = local_path_state
        # this needs state
        self._simple_setup = self._init_simple_setup(land_multi_polygon)

        self.solved = self._simple_setup.solve(time=MAX_SOLVER_RUN_TIME_SEC)

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
                existing_boat.update_collision_zone(ais_ship=ship)
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

    @staticmethod
    def create_buffer_around_position(position: cs.XY, box_buffer_size: float) -> Polygon:
        """Create a space around the given position. Position is the center of the space and
        is a tuple of x and y. Box_buffer_size represents the size of the buffer around position.
        Used in visualizer and ompl_path.
        """
        space = Point(position.x, position.y).buffer(box_buffer_size, cap_style=3, join_style=2)
        return space

    def get_cost(self) -> float:
        """
        Calculate the total cost of the entire path generated by the OMPL planner.

        The cost is computed as the sum of state costs and motion costs for all segments
        in the solution path. If the solution path does not exist, the method returns
        infinity (`float('inf')`).

        Returns:
            float: The total cost of the entire solution path. Returns `float('inf')`
                   if the solution path does not exist.
        """
        try:
            solution_path = self._simple_setup.getSolutionPath()
        except Exception as e:
            self._logger.debug(f"Solution path does not exist. Exception thrown: {e}")
            return float("inf")

        obj = self._simple_setup.getOptimizationObjective()
        cost = solution_path.cost(obj)
        return cost.value()

    def get_remaining_cost(self, last_lp_wp_index: int, boat_lat_lon: ci.HelperLatLon) -> float:
        """
        Calculate the cost of the remaining path from the boat's current position.

        This mirrors OMPL's PathGeometric::cost() formula which computes:

            initialCost(first) + summation motionCost(s_i, s_{i+1}) + terminalCost(last)

        accumulated via combineCosts.

        In this codebase, last_lp_wp_index is the index of the **last local-path waypoint
        the boat has just traversed** (i.e., the "current" waypoint). The remaining cost is:

        - the *partial* motion cost on the segment from waypoint last_lp_wp_index to
          last_lp_wp_index + 1 (based on how far the boat is from the next waypoint), plus
        - the *full* motion costs of all subsequent segments to the goal, plus
        - the terminal cost at the final state.

        Args:
            last_lp_wp_index (int): Index of the last local-path waypoint the boat has just
                traversed (the current waypoint). The next waypoint to head toward is
                last_lp_wp_index + 1.
            boat_lat_lon (ci.HelperLatLon): The boat's current latitude/longitude.

        Returns:
            float: The remaining cost of the path. Returns float('inf') if the solution path
            does not exist.

        Raises:
            ValueError: If last_lp_wp_index is out of bounds for the solution path.
        """
        try:
            solution_path = self._simple_setup.getSolutionPath()
        except Exception as e:
            self._logger.debug(f"Solution path does not exist. Exception thrown: {e}")
            return float("inf")

        obj = self._simple_setup.getOptimizationObjective()
        states = solution_path.getStates()
        num_states = len(states)

        if last_lp_wp_index == num_states - 1:
            return 0.0
        if num_states < 2:
            return solution_path.cost(obj).value()
        if last_lp_wp_index >= num_states:
            raise ValueError(
                "index out of bound for path; ensure that "
                "the last_lp_wp_index is < number of waypoints in the path"
            )

        cost = obj.identityCost()

        # --- Partial cost for the current segment (last_lp_wp_index -> last_lp_wp_index+1) ---
        seg_start = states[last_lp_wp_index]
        seg_end = states[last_lp_wp_index + 1]

        dx_seg = seg_end.getX() - seg_start.getX()
        dy_seg = seg_end.getY() - seg_start.getY()
        total_seg_dist = math.hypot(dx_seg, dy_seg)

        boat_xy = cs.latlon_to_xy(self.state.reference_latlon, boat_lat_lon)
        dx_boat = boat_xy.x - seg_start.getX()
        dy_boat = boat_xy.y - seg_start.getY()
        dist_boat_to_start = math.hypot(dx_boat, dy_boat)

        if total_seg_dist >= DISTANCE_THRESHOLD and dist_boat_to_start >= DISTANCE_THRESHOLD:
            # Project the boat's position onto the segment vector to compute traveled distance.
            # dot(boat_vec, seg_vec) / ||seg_vec|| gives the scalar projection length
            projected_dist = (dx_seg * dx_boat + dy_seg * dy_boat) / total_seg_dist

            # clamp to [0, total_seg_dist]
            projected_dist = max(0.0, min(projected_dist, total_seg_dist))
            fraction_travelled = projected_dist / total_seg_dist
        else:
            # segment is degenerate or boat is at start
            fraction_travelled = 0.0

        fraction_remaining = 1.0 - fraction_travelled
        full_seg_cost = obj.motionCost(seg_start, seg_end).value()
        cost = obj.combineCosts(cost, base.Cost(fraction_remaining * full_seg_cost))

        # Add full costs for segments strictly after the current one
        for i in range(last_lp_wp_index + 1, num_states - 1):
            cost = obj.combineCosts(cost, obj.motionCost(states[i], states[i + 1]))

        return cost.value()

    def get_path(self) -> ci.Path:
        """Get the collection of waypoints for the boat to follow.

        Returns:
            ci.Path: A collection of lat lon coordinates for the waypoints to follow.
                    First waypoint is the start state (near the boat's position) and final waypoint
                    is the goal state (the next global waypoint).
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

    def _init_simple_setup(self, land_multi_polygon) -> og.SimpleSetup:
        # Create buffered rectangles around sailbot's position and the goal state
        start_position_in_xy = cs.latlon_to_xy(self.state.reference_latlon, self.state.position)
        start_box = self.create_buffer_around_position(start_position_in_xy, self._box_buffer)
        start_x = start_position_in_xy.x
        start_y = start_position_in_xy.y

        # goal is at (0,0) because global waypoint is used as the reference point
        goal_position_in_xy = cs.XY(0, 0)
        goal_polygon = self.create_buffer_around_position(goal_position_in_xy, self._box_buffer)
        goal_x, goal_y = goal_position_in_xy

        # RRT* requires a symmetric state space which is not the default for Dubins State Space
        space = base.DubinsStateSpace(turningRadius=MIN_TURNING_RADIUS_KM, isSymmetric=True)
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
            local_path_state=self.state,
            state_space_xy=state_space,
            land_multi_polygon=land_multi_polygon,
        )

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

        space_information = simple_setup.getSpaceInformation()

        self.state.planner = og.RRTstar(space_information)

        objective = get_sailing_objective(
            space_information,
            simple_setup,
            self.state.heading,
            self.state.speed,
            self.state.wind_direction,
            self.state.wind_speed,
        )

        simple_setup.setOptimizationObjective(objective)
        planner = og.RRTstar(space_information)
        planner.setRange(MAX_EDGE_LEN_KM)
        simple_setup.setPlanner(planner)

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

                    # if isinstance(state, base.State):  # only happens in unit tests
                    #     log_invalid_state(state=cs.XY(state().getX(), state().getY()), obstacle=o) # noqa
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
