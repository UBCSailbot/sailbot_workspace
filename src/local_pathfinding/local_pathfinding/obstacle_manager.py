import pickle
from typing import List, Any
from shapely.geometry import MultiPolygon, Polygon
import local_pathfinding.obstacles as ob
from local_pathfinding.coord_systems import XY



class ObstacleManager:
    """
    Manages obstacles for local path planning
    """

    def __init__(self):
        self.obstacles: List[ob.Obstacle] = []
        self.obstacles: List[ob.Boat] = []
        self.land_obstacle = None
        self.all_land_data = None
        self.use_mock_land = False
        self.mock_land_data = None


