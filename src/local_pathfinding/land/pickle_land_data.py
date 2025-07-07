import os
import pickle
from typing import Any

import geopandas as gpd
import pyproj
from shapely.geometry import MultiPolygon

WGS84 = pyproj.CRS("EPSG:4326")
MERCATOR = pyproj.CRS("EPSG:3857")


def dump_pkl(object: Any, file_path: str):
    """Stores an object on disk in the pkl format.

    Args:
        object (Any): Object to be stored.
        file_path (str): file path to store object.

    Raises:
        RuntimeError: If there are any issues with pickling the object.
    """

    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    try:
        with open(file_path, "wb") as f:
            pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)
    except pickle.PickleError as e:
        raise RuntimeError(
            f"An unexpected error occurred when attempting to store the pkl file: {e}"
        )


def pickle_land():
    """
    Generates the complete land dataset to be used by sailbot and stores it in PKL format for
    long term storage on disk.

    Land data is saved to pkl/land.pkl

    """

    gdf = gpd.read_file("shp/complete_land_data.shp")
    multi_poly = MultiPolygon(list(gdf["geometry"]))

    # repair invalid polygons
    multi_poly = multi_poly.buffer(0)

    dump_pkl(multi_poly, "pkl/land.pkl")


if __name__ == "__main__":
    pickle_land()
