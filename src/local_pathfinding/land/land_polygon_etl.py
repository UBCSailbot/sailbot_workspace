"""
This script is used to combine data from two separate streams into one collection of
land mass polygons.

The two data input streams are:

    1. land_polygons.shp
        A collection of polygons representing coastlines and islands.
        This data is based off of the Open Street Map project and downloaded from
        https://osmdata.openstreetmap.de/

    2. GEBCO_2023.nc
        A netCDF file containing bathymetric data. The data is filtered for depth
        and then polygonized using DBSCAN and shapely.concave_hull
        This data is downloaded from
        https://www.gebco.net/

This script generates two files as final products:

    1. complete_land_data.shp (shape file)
        A shape file containing all land mass polygons from the two input streams.

    2. sindex.pkl (pickle file)
        A pickle file containing a binary encoding of an STRTree object which can be loaded
        and used to run spatial queries on its parent GeoDataFrame which contained all land mass
        polygons at the time of pickling.

The following CLI arguments are available:

        --lat (optional): Latitude range, for selecting a subregion of dataset
        --lon (optional): Longitude range, for selecting a subregion of dataset
        --test (optional): Run in test mode. Uses smaller data sets and modifies
          global variables to speed things up
        --jump (optional): Jump past initial data filtering for debugging
"""

import argparse
import csv
import os
import pickle
import subprocess
import zipfile
from os.path import normpath
from shutil import move
from typing import List

import gdown
import geopandas as gpd
import numpy as np
import pandas as pd
import psutil
import pyproj
import requests
import xarray as xr
from geopandas import GeoDataFrame
from shapely import concave_hull, convex_hull
from shapely.geometry import LineString, MultiPoint, Point, Polygon, box
from shapely.ops import split, transform
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from tqdm import tqdm

DOWNLOAD_ATTEMPTS = 3


# Constants
WGS84 = pyproj.CRS("EPSG:4326")
MERCATOR = pyproj.CRS("EPSG:3857")
MIN_DEPTH = -20  # meters
MAX_HEIGHT = 99999  # meters, include all land for now
GRID_SIZE = 0.1  # degrees lat/lons
DOWNLOAD_ATTEMPTS = 3

# Modes
# run in production mode by default
_mode = "PROD"
_jump = False

# Default Latitude and Longitude ranges for the complete global navigation region
LAT_RANGE = (14.6338, 61.4795)  # S:N
LON_RANGE = (-179.9, -109.335938)  # W:E
DEFAULT_BBOX = box(LON_RANGE[0], LAT_RANGE[0], LON_RANGE[1], LAT_RANGE[1])

# SHAPE FILE PATHS
BASE_SHP_FILE = normpath("shp/land_polygons.shp")
SMALL_SHP_FILE = normpath("shp/ne_10m_land.shp")
BBOX_REGION_FILE = normpath("shp/land_polygons_bbox_region.shp")
COMPLETE_DATA_FILE = normpath("shp/complete_land_data.shp")

# SHAPE FILE URLS
BASE_SHP_URL = "https://osmdata.openstreetmap.de/download/land-polygons-split-3857.zip"
SMALL_SHP_URL = "https://naciscdn.org/naturalearth/10m/physical/ne_10m_land.zip"

# CSV PATHS
# this is the polygon which defines the complete navigation region
# all land obstacles will come from polygons which intersect or are bounded by this polygon
MAP_SEL_POLYGON = normpath("csv/map_sel.csv")
# not to be confused with map_sel.csv
# this polygon is used to filter out inland points from the bathymetric dataset
INLAND_FILTER_POLYGON = normpath("csv/inland_polygon.csv")

# NETCDF PATHS
NETCDF_FILE = normpath("netcdf/GEBCO_2023.nc")
NETCDF_SMALL_FILE = normpath("netcdf/salish_sea.nc")

# NETCDF URLS
NET_CDF_URL = "https://www.bodc.ac.uk/data/open_download/gebco/gebco_2023/zip/"
NETCDF_SMALL_URL = "https://drive.google.com/uc?id=11Le2e8sz4xIor4bWsrGSd_ovYgUBTR6b"

# PKL PATHS
SINDEX_FILE = normpath("pkl/sindex.pkl")  # spatial index of final land mass data set
GDF_SPF_FILE = normpath("pkl/gdf_spf.pkl")  # gdf after first spatial filter
GDF_FILTER_FILE = normpath("pkl/gdf_filter.pkl")  # gdf containing polygons to filter against

def dump_pkl(object: any, file_path: str):
    # creating a pickler once, when everything is being pickled
    # will likely be more efficient
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> any:
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

    def ok(self, msg):
        level = self.levels.get("OK")
        print(level + msg + self.reset)

    def warn(self, msg):
        level = self.levels.get("WARN")
        print(level + msg + self.reset)

    def error(self, msg):
        level = self.levels.get("ERROR")
        print(level + msg + self.reset)


def dump_pkl(object: any, file_path: str):
    # creating a pickler once, when everything is being pickled
    # will likely be more efficient
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> any:
    with open(file_path, "rb") as f:
        return pickle.load(f)


def install_packages(requirements_file):
    try:
        # Use subprocess to call pip install
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", requirements_file])
        print(f"All packages from {requirements_file} have been installed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while trying to install the packages: {e}")


def download_zip(url: str, file_name: str, dir: str):
    """
    Saves a file from a URL to the specified directory with the specified name.
    Then calls the unzip() function.

    Args:
        - url (str): The URL of the file to be downloaded.
        - file_name (str): The file name to save the downloaded file to.
        - dir (str): The directory to save the downloaded file to.
    """
    logger = Logger()
    os.makedirs(dir, exist_ok=True)

    tries = DOWNLOAD_ATTEMPTS

    while tries > 0:

        try:
            # download file in chunks of 10MB
            response = requests.get(
                url, stream=True
            )  # stream to avoid loading entire file into memory
            path = os.path.join(dir, file_name)
            with open(path, "wb") as f:
                for chunk in tqdm(
                    response.iter_content(chunk_size=10 * 1024**2),
                    desc=f"Downloading {file_name} to {dir}...",
                    total=int(int(response.headers.get("content-length", 0)) / (10 * 1024**2) + 1),
                ):
                    f.write(chunk)
            # extract files to shp folder
            print(f"Extracting {file_name} to {dir}...")
            unzip(path, extract_to=dir)
            break

        except requests.exceptions.RequestException as e:
            logger.error(f"Failed to download {file_name} from {url}.")
            print(e)
            print("Retrying...")
            tries -= 1
            if tries == 0:
                exit(e)


def flatten_dir(directory):
    """
    Flatten a directory structure by moving all files to the parent directory and then removing
    any subdirectories

    Args:
        - directory (str): The directory to be flattened.
    """
    for root, dirs, files in os.walk(directory):
        # Move all files to the parent directory
        for file in files:
            src = os.path.join(root, file)
            dst = os.path.join(directory, file)
            move(src, dst)
    for root, dirs, files in os.walk(directory, topdown=False):
        for dir in dirs:
            os.rmdir(os.path.join(root, dir))


def gd_download(url: str, file_name: str):
    logger = Logger()
    tries = DOWNLOAD_ATTEMPTS
    while tries > 0:
        try:
            gdown.download(url, file_name, quiet=False)
            break
        except gdown.exceptions.FileURLRetrievalError:
            logger.error(f"Failed to download {file_name} from {url}.")
            print("Retrying...")
            tries -= 1


def unzip(zip_file, extract_to):
    """
    Extracts the contents of a zip file to a specified directory,
    then deletes the original zip file.

    Args:
        - zip_file (str): The file path of the zip file to be unzipped.
        - extract_to (str): The directory to unzip the file to.
    """
    with zipfile.ZipFile(zip_file, "r") as zip_ref:
        zip_ref.extractall(extract_to)
    os.remove(zip_file)


class FailedPolygonError(Exception):
    """
    Raised when a polygonization process on a chunk fails.
    """

    pass


class FailedBathyDataError(Exception):
    """
    Raised when the bathymetric data process fails.
    """

    pass


def main():

    # ----------------------------------------SETUP ----------------------------------------------
    # Create a logger
    logger = Logger()

    # Setup environment variables
    # the Loky function that tries to do this fails, so set it here
    cores = psutil.cpu_count(logical=False)
    os.environ["LOKY_MAX_CPU_COUNT"] = str(cores)

    # Parse Arguments
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
    parser.add_argument("--test", help="Run in test mode.", action="store_true")

    parser.add_argument("--jump", help="Jump past intitial data filtering", action="store_true")

    parser.add_argument("--grid", help="The size of grid cells for chunking data", type=float)

    args = parser.parse_args()

    # Check for process dependent files
    # download data files if they are not present
    if not os.path.exists(BASE_SHP_FILE):
        logger.warn(f"{BASE_SHP_FILE} not found.")
        download_zip(url=BASE_SHP_URL, file_name="base_shp.zip", dir="shp")

    if not os.path.exists(SMALL_SHP_FILE):
        logger.warn(f"{SMALL_SHP_FILE} not found.")
        download_zip(url=SMALL_SHP_URL, file_name="small_shp.zip", dir="shp")

    # flatten the shp directory
    flatten_dir("shp")

    if not os.path.exists(NETCDF_FILE):
        logger.warn(f"{NETCDF_FILE} not found.")
        download_zip(url=NET_CDF_URL, file_name="gebco_2023.zip", dir="netcdf")

    if not os.path.exists(NETCDF_SMALL_FILE):
        logger.warn(f"{NETCDF_SMALL_FILE} not found. Attempting to download.")
        gd_download(NETCDF_SMALL_URL, NETCDF_SMALL_FILE)

    # Determine which mode to run the script in
    if args.test:
        # Run the script in test mode
        logger.ok(msg="Running in TEST mode")
        global _mode
        _mode = "TEST"

        # check paths of existing dependent files
        paths = [
            BASE_SHP_FILE,
            MAP_SEL_POLYGON,
            INLAND_FILTER_POLYGON,
            NETCDF_FILE,
            NETCDF_SMALL_FILE,
            SMALL_SHP_FILE,
        ]
        for path in paths:
            assert os.path.exists(path), f"File not found: {path}"

        # Override data file paths to small test size files
        base_shp = SMALL_SHP_FILE
        netCDF_file = NETCDF_SMALL_FILE

    else:
        # Run in production mode
        logger.ok("Running in PRODUCTION mode")
        base_shp = BASE_SHP_FILE
        netCDF_file = NETCDF_FILE

    # Set grid size
    if args.grid:
        global GRID_SIZE
        GRID_SIZE = args.grid
    # ----------------------------------------END SETUP -------------------------------------------

    # Lat/Lon ranges
    if args.lat and args.lon:
        print("got lat and lon ranges, creating bbox..")
        lat_range = args.lat
        lon_range = args.lon
        bbox = box(args.lon[0], args.lat[0], args.lon[1], args.lat[1])

    elif (args.lat is None) + (args.lon is None) == 1:  # if only one is specified
        logger.error("Both latitude and longitude ranges must be specified.")

    else:
        logger.warn("No lat and lon ranges specified, using default values to create bbox..")
        bbox = DEFAULT_BBOX
        lat_range = LAT_RANGE
        lon_range = LON_RANGE

    # Determine if the script should jump past the initial data filtering
    if args.jump:
        global _jump
        _jump = True
        logger.warn(
            f"Skipping initial data filtering. Loading intermediate results from {GDF_FILTER_FILE}"
        )
        gdf = load_pkl(GDF_FILTER_FILE)

    # JUMP OVER THIS ------------------------------------------------------------------------------

    else:
        # convert bounding box to mercator projection
        bbox = gpd.GeoSeries([bbox], crs=WGS84).to_crs(MERCATOR).iloc[0]

        # read in all land polygons inside the bounding box
        print(f"reading in land polygons from {base_shp}...")
        gdf = gpd.read_file(base_shp, bbox=bbox)

        print(f"Saving selected region to {BBOX_REGION_FILE}...")
        # store bbox selected region back into a shape file
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

        # TODO if output is not correct, check if this transformation can be done with
        # geopandas instead
        # It thin with to_crs() it also failed on the part of map_sel that crosses the IDL
        projection = pyproj.Transformer.from_proj(WGS84, MERCATOR, always_xy=True).transform
        map_sel_east = transform(projection, map_sel_east)

        # load back in our subset of the region with our map selection mask applied
        print(
            f"reading in land polygons from {BBOX_REGION_FILE} with map selection mask from "
            f"{MAP_SEL_POLYGON} applied..."
        )
        gdf = gpd.read_file(BBOX_REGION_FILE, mask=map_sel_east)
        print(len(gdf["geometry"]), f" land polygons loaded from {BBOX_REGION_FILE}.")

        if _mode == "TEST":
            logger.warn(f"TEST MODE: removing {BBOX_REGION_FILE}")
            remove_shape(BBOX_REGION_FILE)

        print("Starting Bathymetric Data Processing...")
        # convert gdf geometry to WSG84
        gdf.to_crs(WGS84, inplace=True)

        logger.warn(f"Saving intermediate results to {GDF_FILTER_FILE}...")
        dump_pkl(gdf, GDF_FILTER_FILE)

    # JUMP TO HERE --------------------------------------------------------------------------------

    try:
        # obtain all polygons from the bathymetric data set
        gdf_bathy = get_bathy_gdf(
            gdf_filter=gdf, lat_range=lat_range, lon_range=lon_range, netcdf=netCDF_file
        )
        logger.ok("Bathymetric Data Processing Complete")

        print("Merging datasets...")
        # merge coastline and bathymetric polygon sets
        gdf_combined = pd.concat([gdf, gdf_bathy], ignore_index=True)

    except FailedBathyDataError:
        logger.error("Bathymetric data processing failed. Failed to polygonize a chunk")
        exit()

    except Exception as e:
        logger.error(f"Bathymetric data processing failed with an unexpected error: {e}")

    # create spatial index object
    sindex = gdf_combined.sindex

    # dummy query to ensure sindex is fully instantiated
    point = Point(-122.743184, 48.268958)
    sindex.query(geometry=point.buffer(distance=0.001, cap_style=3))

    # send sindex to PKL
    dump_pkl(sindex, SINDEX_FILE)
    # send gdf_combined to shp
    # fiona will be used to load the file again, so its the specified engine
    gdf_combined.to_file(filename=COMPLETE_DATA_FILE, engine="fiona")

    logger.ok(
        f"The complete land mass data set has successfully been created and saved as"
        f"'{COMPLETE_DATA_FILE}'."
    )
    logger.ok(f"The corresponding spatial index has been saved as '{SINDEX_FILE}'.")
    logger.ok("Done")

    return


def box_fix(gdf: GeoDataFrame) -> GeoDataFrame:
    """
    Constructs a box around any Point in gdf for which the entry in the 'cluster' column is -1
    The idea is to add more points around any lone or double points so that DBSCAN can
    successfully add that point to a cluster which can be then converted into a Polygon.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame with the columns:
          'geometry' (Points), 'elevation', and 'cluster'

    Returns:
        - gdf_copy (GeoDataFrame): A copy of the original data frame with extra points added
          around any unclustered points and the 'cluster' column removed.
          gdf_copy is ready for the second pass with DBSCAN.
    """
    gdf_copy = gdf.copy()

    # remove the cluster column as it will be added again on the second pass with DBSCAN
    gdf_copy.drop(columns=["cluster"], inplace=True)

    # Extract only the points which were not clustered by DBSCAN
    unclustered_points = gdf[gdf["cluster"] == -1]

    lats = []
    lons = []

    for _, row in unclustered_points.iterrows():

        # Create a box around each unclustered point

        box_points = box_pts(row.geometry)

        lats.extend([point[1] for point in box_points])
        lons.extend([point[0] for point in box_points])

    # add dummy depths for new points since
    # gdf should be depth filtered by the time this function is called
    depths = np.zeros(len(lats))

    geometry = gpd.points_from_xy(lons, lats)

    gdf_tail = GeoDataFrame(geometry=geometry, columns=["geometry"])

    gdf_tail["depth"] = depths

    return pd.concat([gdf_copy, gdf_tail], ignore_index=True)


def box_pts(pt: Point, w: float = 0.001) -> List[Point]:
    """
    Returns a list of points which form a box around the point pt.

    Args:
        - pt (Point): A point around which to build a box of points.
        - w (float): Width of the box

    Returns:
        - points (List[Point]): A list of the points which form a box around the point pt.
    """

    box = pt.buffer(w, cap_style=3)

    return box.exterior.coords


def get_bathy_gdf(
    gdf_filter: GeoDataFrame, lat_range: tuple, lon_range: tuple, netcdf: str
) -> GeoDataFrame:
    """
    Generates a GeoDataFrame containing polygons representing the bathymetric data set.
    Points are extracted from the bathymetric data set and then run through a depth filter and
    two spatial filters.
    The points are then polygonized using DBSCAN and shapely.concave_hull

    Depth Filtering:
        - Any points which are not within the depth range [MIN_DEPTH, MAX_HEIGHT] are removed.

    Spatial Filtering:
        - Any points which are contained by the polygons in gdf_filter.geometry
          and prune_poly are removed.

    Args:
        - gdf_filter (GeoDataFrame): A GeoDataFrame containing polygons which represent the
          coastlines of the navigation region.
        - lat_range (tuple): A tuple containing the minimum and maximum latitudes of the
          navigation region.
        - lon_range (tuple): A tuple containing the minimum and maximum longitudes of the
          navigation region.
        - netCDF_file (str): The file path to the netCDF file containing the bathymetric data set.

    Returns:
        - gdf (GeoDataFrame): A GeoDataFrame containing polygons representing the
          bathymetric data set.
    """
    # create logger
    logger = Logger()

    if _mode == "TEST":
        # speed up the process by using a larger grid size
        global GRID_SIZE
        GRID_SIZE = 1  # degrees lat/lons

    if _jump:
        logger.warn(
            "Entered get_bathy_pts() Jumping past initial data filtering. "
            f"Loading intermediate results from {GDF_SPF_FILE}..."
        )
        gdf_spatial_filtered = load_pkl(GDF_SPF_FILE)

    else:
        # open data stream
        # file is not loaded into memory yet
        with xr.open_dataset(netcdf) as data:

            # only select points within specified region,
            # do not filter for depth at this stage, as it loads entire file into memory
            lat_range_slice = slice(*lat_range)
            lon_range_slice = slice(*lon_range)
            sliced_data = data.sel(lat=lat_range_slice, lon=lon_range_slice)

            # data is transferred to an ndarray array for depth filtering as it is more performant
            elevation = sliced_data.elevation.data
            lats = sliced_data.lat.data
            lons = sliced_data.lon.data

        logger.ok(f"Bathymetric data loaded. Loaded {len(elevation)*len(elevation[0])} points.")
        print("Starting depth filtering...")

        dpts = np.empty((len(elevation), len(elevation[0]), 3), dtype=np.float32)

        # populate dpts with unfiltered data set
        dpts[:, :, 0] = lats[:, None]
        dpts[:, :, 1] = lons[None, :]
        dpts[:, :, 2] = elevation

        # DEPTH FILTERING
        # filter array with a mask
        mask = np.logical_and(dpts[:, :, 2] >= MIN_DEPTH, dpts[:, :, 2] <= MAX_HEIGHT)
        dpts_filtered = dpts[mask]  # dpts_filtered is a 2D array

        del dpts  # free up memory asap

        # START SPATIAL FILTERING
        logger.ok("Depth filtering complete. ")
        print(f"Transferring {len(dpts_filtered)} data points to a GeoDataFrame...")
        # Transfer filtered points to a GeoDataFrame
        latitude = dpts_filtered[:, 0]
        longitude = dpts_filtered[:, 1]
        geometry = gpd.points_from_xy(longitude, latitude)
        gdf_pts = GeoDataFrame(geometry=geometry, columns=["geometry"])

        print(f"Starting spatial filtering on {len(gdf_pts['geometry'])} points...")
        # Filter out inland areas to cut down the number of points
        # Load in a custom Polygon representing inland area to be pruned from the data
        with open(INLAND_FILTER_POLYGON, "r") as f:
            reader = csv.reader(f)
            # skip header
            reader.__next__()
            prune_poly = Polygon([[float(row[1]), float(row[0])] for row in reader])

        gdf_spatial_filtered = spatial_filter(gdf=gdf_pts, geometry=prune_poly)

        logger.ok(
            f"First spatial filter complete. "
            f"{len(gdf_spatial_filtered['geometry'])} points remain. "
            f"Saving intermediate results to {GDF_SPF_FILE}..."
        )

        # dump to pkl create load point
        dump_pkl(gdf_spatial_filtered, GDF_SPF_FILE)

    # split the data into chunks
    chunked_array = points_to_chunked_array(
        lat_range=lat_range,
        lon_range=lon_range,
        gdf=gdf_spatial_filtered,
        grid_size=GRID_SIZE,
        gdf_filter=gdf_filter,
    )
    # END SPATIAL FILTERING

    print(
        f"Created chunked array from bathymetric points. "
        f"Running Clustering on {len(chunked_array)} chunks..."
    )

    # START POLYGONIZATION
    print(" Starting Polygonization...")
    # all polygons generated by the polygonize_chunks() will be stored in this list
    polygons = []
    # to track the chunks which could not be polygonized
    failures = 0
    scaler = StandardScaler()

    for i, chunk in tqdm(
        enumerate(chunked_array),
        desc="Processing Bathymetric Data Chunks",
        total=len(chunked_array),
    ):

        if len(chunk) == 0:  # empty chunk
            continue

        # create a GDF from the chunked ndarray
        latitude = chunk[:, 0]
        longitude = chunk[:, 1]
        geometry = gpd.points_from_xy(longitude, latitude)
        gdf_chunk = GeoDataFrame(geometry=geometry, columns=["geometry"])

        gdf_polygons = None

        try:
            # polygonize the chunk with default eps and ratio
            gdf_polygons = polygonize_chunks(gdf_chunk, scaler=scaler)

        except FailedPolygonError:

            try:
                # lower eps
                gdf_polygons = polygonize_chunks(gdf_chunk, scaler=scaler, eps=0.0005)

            except FailedPolygonError:

                try:
                    # lower eps and raise ratio
                    gdf_polygons = polygonize_chunks(
                        gdf_chunk, scaler=scaler, eps=0.0005, ratio=0.1
                    )

                except FailedPolygonError:
                    # Still failed to polygonize
                    # try raising ratio even more in the previous try/except layer
                    logger.error(
                        f"Could not polygonize chunk #{i}."
                        "results will not be reliable. Aborting bathymetric data processing."
                    )
                    raise FailedBathyDataError

        if gdf_polygons is not None:
            polygons.extend(gdf_polygons)
        else:
            logger.error(
                f"Tried to add polygons from chunk #{i} to the total polygon list "
                "but gdf_polygons is None. This should never happen"
            )
            exit()

    logger.ok("Polygonization Complete")

    # END POLYGONIZATION

    if failures > 0:
        logger.error(f"Task failed. {failures} chunks could not be polygonized.")
    else:
        logger.ok(f"All chunks were successful. {len(polygons)} polygons generated.")

    return GeoDataFrame(geometry=polygons)


def polygonize_chunks(
    gdf: GeoDataFrame, scaler: StandardScaler, eps: float = 0.001, ratio: float = 0.05
) -> List[Polygon]:
    """
    Clusters a set of coordinates using DBSCAN and
    polygonizes the clusters using shapely.concave_hull.

    Important Note: The default eps and ratio values were determined through trial and error.
                    * The DBSCAN algorithm is very sensitive to the eps value, too small of an eps
                    will cause the number of clusters to explode as eps will be below the unit
                    distance between points in the data set, making every point its own cluster.

                    * The concave_hull algorithm is also sensitive to the ratio value, any value
                    below 0.05 will create polygons with a lot of empty space inside. Higher values
                    will create lower detailed/more convex hulls.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of coordinates to be polygonized.
        - scaler (StandardScaler): A StandardScaler object which will be used to scale the
          coordinates before clustering.
        - eps (float): The maximum distance between two samples for one to be considered as in the
          neighborhood of the other. Used by the DBSCAN algorithm.
        - ratio (float in [0,1]): The concavity ratio of the hull. ratio = 1 gives convex_hull

    Returns:
        - gdf_polygons (List[Polygon]): A list of polygons generated from the clusters.

    Raises:
        FailedPolygonError: Raised when polygonization fails for any reason
    """

    coordinates = np.array([(point.x, point.y) for point in gdf["geometry"]])

    if len(coordinates) == 0:
        return None

    # FIRST DBSCAN PASS
    scaled_coordinates = scaler.fit_transform(coordinates)
    db = DBSCAN(eps=eps, min_samples=3, metric="haversine").fit(np.radians(scaled_coordinates))
    gdf["cluster"] = db.labels_

    # SECOND DBSCAN PASS
    if -1 in db.labels_:
        gdf = box_fix(gdf)
        # coordinates = MultiPoint([point for point in gdf_near["geometry"]])
        coordinates = np.array([(point.x, point.y) for point in gdf["geometry"]])
        scaled_coordinates = scaler.fit_transform(coordinates)

        db = DBSCAN(eps=eps, min_samples=3, metric="haversine").fit(np.radians(scaled_coordinates))
        gdf["cluster"] = db.labels_

    # sep clusters into separate GeoDataFrames
    gdf_clusters = [gdf[gdf["cluster"] == i] for i in range(max(gdf["cluster"]) + 1)]

    # Polygonize
    try:
        # Check how many polygons we should obtain
        # this provides a minimum count as convex_hull will not lose any polygons
        convex_hulls = list(
            map(
                lambda cluster: convex_hull(
                    MultiPoint(cluster["geometry"]),
                ),
                gdf_clusters,
            )
        )

        gdf_polygons = list(
            map(
                lambda cluster: concave_hull(
                    MultiPoint(cluster["geometry"]),
                    ratio=ratio,
                ),
                gdf_clusters,
            )
        )

        # Ensure no clusters were lost by the concave_hull operation
        if len(convex_hulls) != len(gdf_polygons):
            raise FailedPolygonError("Some polygons were lost during polygonization")

    except Exception as e:

        raise FailedPolygonError from e

    return gdf_polygons


def points_inside(
    gdf: GeoDataFrame, bbox: Polygon, gdf_filter: GeoDataFrame = None
) -> List[Point]:
    """
    Returns all points in gdf that are inside bbox. If gdf_filter is not None, the points that
    are within any polygon stored in gdf_filter are removed.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of points.
        - bbox (Polygon): A Polygon which defines the region to be queried.
        - gdf_filter ([Optional] GeoDataFrame): A GeoDataFrame containing polygons which
          represent the coastlines
          used to filter the points in gdf.


    Returns:
        - matches (List[Point]): A list of all points in gdf that are inside bbox.
          If gdf_filter is not None, the points are filtered against gdf_filter polygons.
    """
    # create logger
    logger = Logger()

    # find all points in gdf that are inside bbox
    bbox_pts_idx = list(gdf.sindex.query(geometry=bbox))
    gdf_bbox_pts = gpd.GeoDataFrame(
        geometry=gdf.iloc[bbox_pts_idx]["geometry"], columns=["geometry"]
    )
    gdf_bbox_pts.reset_index(
        drop=True, inplace=True
    )  # so that the indices align with `idx_to_drop` later

    if gdf_filter is not None and _mode != "TEST":

        # find all polygons in gdf_filter that intersect bbox
        gdf_polygons = gdf_filter.iloc[gdf_filter.sindex.query(bbox, predicate="intersects")]

        # find all points in gdf_bbox_pts that are inside any polygon from gdf_filter
        # using sjoin might speed this up more
        idx_to_drop = list(
            np.fromiter(
                map(
                    lambda poly: list(
                        gdf_bbox_pts.sindex.query(geometry=poly, predicate="contains")
                    ),
                    gdf_polygons["geometry"],
                ),
                dtype=np.int,
            ).flatten()
        )

        # remove duplicates
        idx_to_drop = list(set(idx_to_drop))
        try:
            gdf_bbox_pts = gdf_bbox_pts.drop(
                idx_to_drop
            )  # remove points that are inside any polygon form gdf_filter

        except KeyError as e:

            logger.error(
                f"attempted to drop indices: {idx_to_drop} from dataframe with"
                f"indicies: {gdf_bbox_pts.index}."
            )
            exit(e)

    return gdf_bbox_pts["geometry"]


def points_to_chunked_array(
    lat_range: tuple,
    lon_range: tuple,
    gdf: GeoDataFrame,
    grid_size: float = 1,
    gdf_filter: GeoDataFrame = None,
) -> List[np.ndarray]:
    """
    Creates a ragged array of ndarrays containing [lon,lat] points (from gdf) that are
    geographically located inside a corresponding grid cell of the subdivided region defined by
    lat_range, lon_range,
    and grid_size.

    Each row of the array is referred to as a chunk of the total data set.

    Note: while it's not required, the points in gdf.geometry are expected to be ordered as if they
    were in a 3D meshgrid of points that was flattened  into a 2D array of points.
    This is actually why this function is needed, because such a layout of points is not
    conductive to the chunking -> clustering -> polygonization process.

    Args:
        - lat_range (tuple): A tuple containing the minimum and maximum latitudes of the region.
        - lon_range (tuple): A tuple containing the minimum and maximum longitudes of the region.
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of points.
        - grid_size (float): The size of the grid cells in degrees.
        - gdf_filter (GeoDataFrame): A GeoDataFrame containing polygons which represent the
          coastlines
          used to filter the points in gdf.

    Returns:
        - pts_array_list (List[np.ndarray]): A list of ndarrays containing [lon,lat] points that
          are geographically located inside a corresponding grid cell.
          If gdf_filter is not None, the points are filtered against gdf_filter polygons.
    """
    lat_range = np.linspace(
        lat_range[0], lat_range[1], int((lat_range[1] - lat_range[0]) / grid_size)
    )
    lon_range = np.linspace(
        lon_range[0], lon_range[1], int((lon_range[1] - lon_range[0]) / grid_size)
    )
    lat_mesh, lon_mesh = np.meshgrid(lat_range, lon_range)

    # merge mesh grids to create point grid
    grid_pts = np.stack((lat_mesh, lon_mesh), axis=2)

    pts_array_list = []

    # lons go from 0-n-1
    # lats go from 0-n-1
    for i in tqdm(
        range(len(grid_pts) - 1),
        desc="Grouping bathymetric points by grid cell and running second spatial filter.",
    ):
        for j in range(len(grid_pts[0]) - 1):
            # (lon1, lat1, lon2, lat2)
            bbox = box(
                grid_pts[i][j][1],
                grid_pts[i][j][0],
                grid_pts[i + 1][j + 1][1],
                grid_pts[i + 1][j + 1][0],
            )
            pts_array_list.append(
                np.array(
                    [
                        [point.x, point.y]
                        for point in points_inside(gdf=gdf, bbox=bbox, gdf_filter=gdf_filter)
                    ]
                )
            )

    return pts_array_list


def remove_shape(file_path: str):
    """
    Removes the shape file and its associated files from the file system.

    Args:
        - file_path (str): The file path of the shape file to be removed.
    """
    logger = Logger()

    if os.path.exists(file_path):
        prefix = file_path.split(".")[0]
        for ext in [".cpg", ".dbf", ".prj", ".shx", ".shp"]:
            file = prefix + ext
            if os.path.exists(file):
                os.remove(file)
    else:
        logger.warn(f"{file_path} not found. Did not remove anything.")


def spatial_filter(gdf: GeoDataFrame, geometry: Polygon) -> GeoDataFrame:
    """
    Returns a GeoDataFrame with all points inside `geometry` removed.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of points.
        - geometry (Polygon): shapely polygon to filter points against.
          Points in the polygon's interior are filtered out.

    Returns:
        - gdf (GeoDataFrame): A GeoDataFrame with all points inside `geometry` removed.
    """
    return gdf.drop(list(gdf.sindex.query(geometry=geometry)))


if __name__ == "__main__":
    main()
