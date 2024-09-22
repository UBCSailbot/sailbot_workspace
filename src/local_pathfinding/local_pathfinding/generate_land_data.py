"""
This script is used to generate a complete land mass data set for the navigation region.
The script computes all buffers around land polygons, to prevent the need for costly buffer
calculations at Sailbot navigation  runtime.

The data source is:

    1. land_polygons.shp
        A collection of polygons representing coastlines and islands.
        This data is based off of the Open Street Map project and downloaded from
        https://osmdata.openstreetmap.de/
        The data may be used for ANY purpose under the Open Database License (ODbL) v1.0.
        https://www.openstreetmap.org/copyright

This script generates two files as final products:

    1. complete_land_data.shp
        A binary file containing all land mass polygons from source data file

    2. sindex.pkl
        A PKL file containing a binary encoding of an STRTree object which can be loaded
        and used to run spatial queries on its parent GeoDataFrame which contained all land mass
        polygons at the time of pickling

The following CLI arguments are available:

        --lat (optional): Latitude range, for selecting a subregion of dataset
        --lon (optional): Longitude range, for selecting a subregion of dataset
"""

import argparse
import csv
import os
import pickle
import subprocess
import sys
from os.path import normpath
from typing import Any

import geopandas as gpd
import pyproj
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import split, transform
from tqdm import tqdm

# Constants
WGS84 = pyproj.CRS("EPSG:4326")
MERCATOR = pyproj.CRS("EPSG:3857")

# Default Latitude and Longitude ranges for the complete global navigation region
LAT_RANGE = (14.6338, 61.4795)  # S:N
LON_RANGE = (-179.9, -109.335938)  # W:E
DEFAULT_BBOX = box(LON_RANGE[0], LAT_RANGE[0], LON_RANGE[1], LAT_RANGE[1])
LAND_BUFFER = 23_150  # 12.5 nautical miles in meters

REQUIREMENTS_FILE = normpath(
    "/workspaces/sailbot_workspace/src/local_pathfinding/requirements.txt"
)

# SHAPE FILE PATHS
BASE_SHP_FILE = normpath(
    "/workspaces/sailbot_workspace/src/local_pathfinding/land/shp/land_polygons.shp"
)
BBOX_REGION_FILE = normpath(
    "/workspaces/sailbot_workspace/src/local_pathfinding/land/shp/land_polygons_bbox_region.shp"
)
COMPLETE_DATA_FILE = normpath(
    "/workspaces/sailbot_workspace/src/local_pathfinding/land/shp/complete_land_data.shp"
)
BASE_SHP_URL = "https://osmdata.openstreetmap.de/download/land-polygons-split-3857.zip"

# this irregularly shaped polygon defines the complete expected navigation region
# all land obstacles will come from polygons which intersect this polygon
MAP_SEL_POLYGON = normpath(
    "/workspaces/sailbot_workspace/src/local_pathfinding/land/csv/map_sel.csv"
)
# spatial index of final land mass data set
SINDEX_FILE = normpath("/workspaces/sailbot_workspace/src/local_pathfinding/land/pkl/sindex.pkl")


def dump_pkl(object: Any, file_path: str):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> Any:
    with open(file_path, "rb") as f:
        return pickle.load(f)


# ----------------------------------------UTILS----------------------------------------------
class Logger:
    """
    Little custom colored logger to clean up print statements.
    """

    def __init__(self):
        self.levels = {
            "ERROR": "\033[91m",
            "OK": "\033[92m",
            "WARN": "\033[93m",
        }
        self.reset = "\033[0m"

    def info(self, msg):
        level = self.levels.get("OK")
        print(level + msg + self.reset)

    def warn(self, msg):
        level = self.levels.get("WARN")
        print(level + msg + self.reset)

    def error(self, msg):
        level = self.levels.get("ERROR")
        print(level + msg + "Exiting" + self.reset)
        exit(1)


def install_packages(requirements_file):
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", requirements_file])
        print(f"All packages from {requirements_file} have been installed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while trying to install the packages: {e}")


def main():

    # ----------------------------------------SETUP ----------------------------------------------
    logger = Logger()

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lat",
        help="Latitude range, for selecting a subregion of dataset.",
        type=float,
        nargs="*",
    )
    parser.add_argument(
        "--lon",
        help="Longitude range, for selecting a subregion of dataset.",
        type=float,
        nargs="*",
    )

    args = parser.parse_args()

    if not os.path.exists(BASE_SHP_FILE):
        logger.error(f"{BASE_SHP_FILE} not found.")

    if not os.path.exists(MAP_SEL_POLYGON):
        logger.error(f"{MAP_SEL_POLYGON} not found.")

    base_shp = BASE_SHP_FILE

    # ----------------------------------------END SETUP -------------------------------------------

    if args.lat and args.lon:
        print("got lat and lon ranges, creating bbox..")
        bbox = box(args.lon[0], args.lat[0], args.lon[1], args.lat[1])

    elif (args.lat is None) + (args.lon is None) == 1:  # if only one is specified
        logger.error("Both latitude and longitude ranges must be specified.")

    else:
        logger.warn("No lat and lon ranges specified, using default values to create bbox..")
        bbox = DEFAULT_BBOX

    # convert bounding box to mercator projection
    bbox = gpd.GeoSeries([bbox], crs=WGS84).to_crs(MERCATOR).iloc[0]

    print(f"reading in land polygons from {base_shp}...")
    gdf = gpd.read_file(base_shp, bbox=bbox)

    print(f"Saving selected region to {BBOX_REGION_FILE}...")
    # this is to keep only the polygons that intersect with the bounding box
    # and discard the rest
    # it makes the next filtering step faster
    gdf.to_file(BBOX_REGION_FILE)

    # Load in a custom Polygon which hugs coastline, to prune off unneeded inland polygons
    with open(MAP_SEL_POLYGON, "r") as f:
        reader = csv.reader(f)
        # skip header
        reader.__next__()
        map_sel = Polygon([[float(row[1]), float(row[0])] for row in reader])

    # Slice off section of map_sel west of the International Date Line
    # transforming a polygon that crosses the IDL has undesired results
    IDL = LineString([(-180, 90), (-180, -90)])
    map_sel_east = split(map_sel, IDL).geoms[0]
    # the land dataset is in mercator projection, to select only the data inside of bbox
    # bbox must also be in mercator
    projection = pyproj.Transformer.from_proj(WGS84, MERCATOR, always_xy=True).transform
    map_sel_east = transform(projection, map_sel_east)

    print(
        f"reading in land polygons from {BBOX_REGION_FILE} with map selection mask from "
        f"{MAP_SEL_POLYGON} applied..."
    )
    gdf_complete = gpd.read_file(BBOX_REGION_FILE, mask=map_sel_east)
    print(len(gdf_complete["geometry"]), f" land polygons loaded from {BBOX_REGION_FILE}.")

    print("head of gdf_complete pre-buffering:")
    print(gdf_complete.head)

    unbuffered_polygons = gdf_complete["geometry"].values

    logger.info("Buffering polygons...")
    buffered_polygons = list(
        tqdm(
            map(lambda poly: poly.buffer(LAND_BUFFER, join_style=2), unbuffered_polygons),
            total=len(unbuffered_polygons),
        )
    )

    gdf_complete_buffered = gpd.GeoDataFrame(geometry=buffered_polygons)
    # this will transform the polygons back to WGS84 with units of degrees
    gdf_complete_buffered.to_crs(WGS84, inplace=True)

    print("head of gdf_complete_buffered after transformation to WGS84:")
    print(gdf_complete_buffered.head)

    logger.info("Creating spatial index...")
    sindex = gdf_complete_buffered.sindex

    # dummy query to ensure sindex is fully initialized
    point = Point(-122.743184, 48.268958)
    sindex.query(geometry=point.buffer(distance=0.001, cap_style=3))

    dump_pkl(sindex, SINDEX_FILE)
    # fiona will be used to load the file again, so it is the specified engine
    gdf_complete_buffered.to_file(filename=COMPLETE_DATA_FILE, engine="fiona")

    logger.info(
        f"The complete land mass data set has successfully been created and saved as"
        f"'{COMPLETE_DATA_FILE}'."
    )
    logger.info(f"The corresponding spatial index has been saved as '{SINDEX_FILE}'.")
    logger.info("Done")


if __name__ == "__main__":
    main()
