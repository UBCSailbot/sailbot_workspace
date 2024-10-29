import csv
import os
import pickle
from os.path import normpath
from typing import Any

import geopandas as gpd
import pyproj
from shapely.geometry import LineString, MultiPolygon, Polygon
from shapely.ops import split


def dump_pkl(object: Any, file_path: str):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> Any:
    with open(file_path, "rb") as f:
        return pickle.load(f)


def pickle_land():
    """
    Generates the complete land dataset to be used by sailbot and stores it in PKL format for
    long term storage on disk.

    Land data is saved to:
    /workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/land.pkl

    """

    WGS84 = pyproj.CRS("EPSG:4326")
    MERCATOR = pyproj.CRS("EPSG:3857")
    INLAND_POLYGON = normpath(
        "/workspaces/sailbot_workspace/src/local_pathfinding/land/csv/map_sel.csv"
    )
    BUFFER_AMOUNT = 5000  # meters

    gdf = gpd.read_file(
        "/workspaces/sailbot_workspace/src/local_pathfinding/land/territorial_waters.geojson"
    )

    # the only useful data is in the id and geometry columns and in the first two rows
    gdf = gdf[["id", "geometry"]]
    gdf = gdf.head(2)
    geoms = list(gdf.iloc[0]["geometry"].geoms) + list(gdf.iloc[1]["geometry"].geoms)

    # convert to MultiPolygon for easier geometric manipulation
    multi_poly = MultiPolygon(geoms)
    multi_poly = multi_poly.buffer(0)  # repairs invalid geometry
    # this is to prevent geometry the crosses -180 longitude from being distorted by transforms
    with open(INLAND_POLYGON, "r") as f:
        reader = csv.reader(f)
        reader.__next__()
        map_sel = Polygon([[float(row[1]), float(row[0])] for row in reader])
    IDL = LineString([(-179.5, 90), (-179.5, -90)])
    map_sel_east = split(map_sel, IDL).geoms[0]
    multi_poly_clipped = multi_poly.intersection(map_sel_east)

    # back to gdf for easier coordinate system transformation
    gdf_clipped = gpd.GeoDataFrame(geometry=[multi_poly_clipped], crs=WGS84)
    gdf_clipped.to_crs(MERCATOR, inplace=True)
    gdf_clipped["geometry"] = gdf_clipped.buffer(BUFFER_AMOUNT)
    gdf_clipped.to_crs(WGS84, inplace=True)

    # Land class will deal with Multipolygon
    multi_poly_clipped_buffered = MultiPolygon(list(gdf_clipped.iloc[0]["geometry"].geoms))

    dump_pkl(
        multi_poly_clipped_buffered,
        "/workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/land.pkl",
    )


if __name__ == "__main__":
    pickle_land()
