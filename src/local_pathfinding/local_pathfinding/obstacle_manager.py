import pickle
from typing import List, Any, Union
import local_pathfinding.obstacles as ob
from ompl import base
import local_pathfinding.coord_systems as cs


class ObstacleManager:
    """
    Manages obstacles for local path planning
    """

    def __init__(self):
        self.obstacles: List[ob.Obstacle] = []
        self.boat_obstacles: List[ob.Boat] = []
        self.land_obstacle = None
        self.all_land_data = None
        self.use_mock_land = False
        self.mock_land_data = None

    def load_land_data(self, mock_land_file=None):
        """ Load land data (mock or real) """
        if self.use_mock_land and mock_land_file:
            self.all_land_data = self._load_pkl(mock_land_file)
        else:
            self.all_land_data = self._load_pkl(
                "/workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/land.pkl"
            )

    def load_pkl(self, file_path: str) -> Any:
        with open(file_path, "rb") as f:
            return pickle.load(f)

    def update_obstacles(self, local_path_state, state_space_xy=None):
        """ Updates existing obstacles based on current state (instead of recreating) """
        self.boat_obstacles = []

        # Add boat obstacles
        ais_ships = local_path_state.ais_ships

        if ais_ships:
            for ship in ais_ships:
                self.boat_obstacles.append(
                    ob.Boat(
                        local_path_state.reference_latlon,
                        local_path_state.position,
                        local_path_state.speed,
                        ship,
                    )
                )

        # Add land obstacles
        if state_space_xy is None:
            state_space_latlon = None
        else:
            # convert the current XY state space back to latlon coordinates
            # to generate land obstacles
            state_space_latlon = cs.xy_polygon_to_latlon_polygon(
                reference=local_path_state.reference_latlon, poly=state_space_xy
            )

        # Create or update land obstacle
        if self.land_obstacle is None:
            # First time initialization
            self.land_obstacle = ob.Land(
                reference=local_path_state.reference_latlon,
                sailbot_position=local_path_state.position,
                all_land_data=self.all_land_data,
                state_space=state_space_latlon,
            )
        else:
            # Update existing land obstacle
            self.land_obstacle.update(
                reference=local_path_state.reference_latlon,
                sailbot_position=local_path_state.position,
                state_space=state_space_latlon,
            )

        # Combine all obstacles
        self.obstacles = self.boat_obstacles + [self.land_obstacle]
        return self.obstacles

    def get_obstacles(self):
        return self.obstacles

    def is_state_valid(self, state: Union[base.State, base.SE2StateInternal]) -> bool:
        """Evaluate a state to determine if the configuration collides with an environment
        obstacle.

        Args:
            state (base.SE2StateInternal): State to check.

        Returns:
            bool: True if state is valid, else false.
        """

        for o in self.obstacles:

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
