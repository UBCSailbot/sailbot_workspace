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
        and then polygonized using DBSCAN and alphashape.
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
"""

import argparse
import csv
import os
import pickle
from typing import List

import alphashape
import geopandas as gpd
import numpy as np
import pandas as pd
import pyproj
import tqdm
import xarray as xr
from geopandas import GeoDataFrame
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import split, transform
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

# Constants
WGS84 = pyproj.CRS("EPSG:4326")
MERCATOR = pyproj.CRS("EPSG:3857")
MIN_DEPTH = -20  # meters
MAX_HEIGHT = 99999  # meters, include all land for now
GRID_SIZE = 0.1  # degrees lat/lons

# Default Latitude and Longitude ranges for the complete global navigation region
LAT_RANGE = (14.6338, 61.4795)  # S:N
LON_RANGE = (-179.9, -109.335938)  # W:E
DEFAULT_BBOX = box(LON_RANGE[0], LAT_RANGE[0], LON_RANGE[1], LAT_RANGE[1])

# SHAPE FILE PATHS
BASE_SHP_FILE = "shp/land_polygons.shp"
BBOX_REGION_FILE = "shp/land_polygons_bbox_region.shp"
COMPLETE_DATA_FILE = "shp/complete_land_data.shp"

# CSV PATHS
# this is the polygon which defines the complete navigation region
# all land obstacles will come from polygons which intersect or are bounded by this polygon
MAP_SEL_POLYGON = "/csv/map_sel.csv"
# not to be confused with map_sel.csv
# this polygon is used to filter out inland points from the bathymetric dataset
INLAND_FILTER_POLYGON = "/csv/inland_polygon.csv"

# NETCDF PATHS
NETCDF_FILE = "netcdf/gebco_2023/GEBCO_2023.nc"

# PKL PATHS
SINDEX_FILE = "/pkl/sindex.pkl"


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lat", help="Latitude range, for selecting a subregion of dataset.", type=List[float]
    )
    parser.add_argument(
        "--lon", help="Longitude range, for selecting a subregion of dataset.", type=List[float]
    )
    parser.add_argument("--test", help="Run in test mode.", action="store_true")
    args = parser.parse_args()

    if args.test:
        # Run the script in test mode
        print("Running in test mode...")

        # check paths of existing dependent files
        paths = [BASE_SHP_FILE, MAP_SEL_POLYGON, INLAND_FILTER_POLYGON, NETCDF_FILE]
        for path in paths:
            assert os.path.exists(path), f"File not found: {path}"

        # Override netcdf filepath to small file
        netCDF_file = "netcdf/gebco_2023_small/salish_sea.nc"
        remove_gen_files = True

    else:
        # Run in production mode
        print("Running in production mode...")
        netCDF_file = NETCDF_FILE
        remove_gen_files = False

    if args.lat and args.lon:
        print("got lat and lon ranges, creating bbox..")
        lat_range = args.lat
        lon_range = args.lon
        bbox = box(args.lon[0], args.lat[0], args.lon[1], args.lat[1])

    elif (args.lat is None) + (args.lon is None) == 1:  # if only one is specified
        parser.error("Both latitude and longitude ranges must be specified.")

    else:
        print("No lat and lon ranges specified, using default values to create bbox..")
        bbox = DEFAULT_BBOX
        lat_range = LAT_RANGE
        lon_range = LON_RANGE

    # convert bounding box to mercator projection
    bbox = gpd.GeoSeries([bbox], crs=WGS84).to_crs(MERCATOR).iloc[0]

    # read in all land polygons inside the bounding box
    print(f"reading in land polygons from {BASE_SHP_FILE}...")
    gdf = gpd.read_file(BASE_SHP_FILE, bbox=bbox)

    print(f"saving selected region to {BBOX_REGION_FILE}...")
    # store bbox selected region back into a shape file
    bbox_region_path = BBOX_REGION_FILE
    gdf.to_file(bbox_region_path)

    # Load in a custom Polygon which hugs coastline, to prune off unneccesary inland polygons
    with open(MAP_SEL_POLYGON, "r") as f:
        reader = csv.reader(f)
        # skip header
        reader.__next__()
        map_sel = Polygon([[float(row[1]), float(row[0])] for row in reader])

    # Slice off section of map_sel west of the International Date Line
    # transforming a polygon that crosses the IDL has undesired results
    IDL = LineString([(-180, 90), (-180, -90)])
    map_sel_east = split(map_sel, IDL).geoms[0]

    # TODO check if I can do this with geopandas, might solve my issue?
    projection = pyproj.Transformer.from_proj(WGS84, MERCATOR, always_xy=True).transform
    map_sel_east = transform(projection, map_sel_east)

    # load back in our subset of the region with our map selection mask applied
    print(
        f"reading in land polygons from {BBOX_REGION_FILE} with map selection mask from"
        f"{MAP_SEL_POLYGON} applied..."
    )
    gdf = gpd.read_file(bbox_region_path, mask=map_sel_east)

    if remove_gen_files:
        print(f"Running in test mode. removing {bbox_region_path}")
        os.remove(bbox_region_path)

    print("Starting Bathymetric Data Processing...")
    # obtain all polygons from the bathymetric data set
    gdf_bathy = get_bathy_gdf(
        gdf_filter=gdf, lat_range=lat_range, lon_range=lon_range, netcdf=netCDF_file
    )
    print("Bathymetric Data Processing Complete")

    # finally convert gdf to WSG84
    # crs attr must be set pre to_crs()
    gdf.crs = MERCATOR
    gdf.to_crs(WGS84)

    print("Merging datasets...")
    # merge coastline and bathymetric polygon sets
    gdf_combined = pd.concat([gdf, gdf_bathy], ignore_index=True)

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

    print(
        f"The complete land mass data set has successfully been created and saved as"
        f"'{COMPLETE_DATA_FILE}'."
    )
    print(f"The corresponding spatial index has been saved as '{SINDEX_FILE}'.")

    if remove_gen_files:
        print(f"Running in test mode. removing {COMPLETE_DATA_FILE} and {SINDEX_FILE}")
        os.remove(COMPLETE_DATA_FILE)
        os.remove(SINDEX_FILE)
    return


def get_bathy_gdf(
    gdf_filter: GeoDataFrame, lat_range: tuple, lon_range: tuple, netcdf: str
) -> GeoDataFrame:
    """
    Generates a GeoDataFrame containing polygons representing the bathymetric data set.
    Points are extracted from the bathymetric data set and then run through a depth filter and
    two spatial filters.
    The points are then polygonized using DBSCAN and alphashape.

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

    # Transfer filtered points back to a GeoDataFrame
    latitude = dpts_filtered[:, 0]
    longitude = dpts_filtered[:, 1]
    depth = dpts_filtered[:, 2]

    geometry = gpd.points_from_xy(longitude, latitude)
    gdf_filtered = GeoDataFrame(geometry=geometry, columns=["geometry"])

    # Add depth values as a new column
    gdf_filtered["depth"] = depth

    # Filter out inland areas to cut down the number of points
    # Load in a custom Polygon representing inland area to be pruned from the data
    with open(INLAND_FILTER_POLYGON, "r") as f:
        reader = csv.reader(f)
        # skip header
        reader.__next__()
        prune_poly = Polygon([[float(row[1]), float(row[0])] for row in reader])

    # spatial filtering is done sequentially as spatial filter 2 is more expensive
    gdf_spatial_filtered_1 = spatial_filter(gdf_filtered, prune_poly)
    gdf_spatial_filtered_2 = spatial_filter(
        gdf_spatial_filtered_1, geometry=gdf_filter["geometry"]
    )

    # END SPATIAL FILTERING

    # START POLYGONIZATION

    # split the data into chunks
    chunked_array = points_to_chunked_array(
        lat_range=lat_range, lon_range=lon_range, gdf=gdf_spatial_filtered_2, grid_size=GRID_SIZE
    )
    print(
        f"Created chunked array from bathymetric points."
        f"Running Clustering on {len(chunked_array)} chunks..."
    )

    # all polygons generated by the polygonize_chunks() will be stored in this list
    polygons = []
    # to track the chunks which could not be polygonized
    failures = 0
    scaler = StandardScaler()

    for i, chunk in tqdm(enumerate(chunked_array), desc="Processing Bathymetric Data Chunks"):

        print(f"Processing chunk {i}...")

        if len(chunk) == 0:  # empty chunk
            continue

        print(f"Chunk {i} has {len(chunk)} points. Running DBSCAN...")

        # create a GDF from the chunk(ndarray)
        latitude = chunk[:, 0]
        longitude = chunk[:, 1]
        geometry = gpd.points_from_xy(longitude, latitude)
        gdf_chunk = GeoDataFrame(geometry=geometry, columns=["geometry"])

        gdf_polygons = None

        try:
            # polygonize the chunk
            gdf_polygons = polygonize_chunks(gdf_chunk, scaler=scaler)

        except Exception as e:

            print(chunk)
            print(e)
            failures += 1
            break

        if gdf_polygons is not None:
            polygons.extend(gdf_polygons)  # TODO optimal? what is complexity of extend?

    print("Polygonization Complete")

    # END POLYGONIZATION

    if failures > 0:
        print(f"Number of failed alphashapes: {failures}")
    else:
        print(f"All chunks were successfully polygonized. {len(polygons)} polygons generated.")

    return GeoDataFrame(geometry=polygons)


def dump_pkl(object: any, file_path: str):
    # creating a pickler once, when everything is being pickled
    # will likely be more efficient
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> any:
    # creating a single unpickler instance to use for all unpickling during runtime
    # will likely be more efficient
    with open(file_path, "rb") as f:
        return pickle.load(f)


def polygonize_chunks(
    gdf: GeoDataFrame, scaler: StandardScaler, eps: float = 0.00001, alpha: int = 100
) -> List[Polygon]:
    """
    Clusters a set of coordinates using DBSCAN and polygonizes the clusters using DBSCAN.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of coordinates to be polygonized.
        - scaler (StandardScaler): A StandardScaler object which will be used to scale the
          coordinates before clustering.
        - eps (float): The maximum distance between two samples for one to be considered as in the
          neighborhood of the other.
        - alpha (int): The alpha value to be used in the alphashape algorithm.

    Returns:
        - gdf_polygons (List[Polygon]): A list of polygons generated from the clusters.
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
        print("Some points were not clustered. Running DBSCAN again...")
        gdf = box_fix(gdf)
        # coordinates = MultiPoint([point for point in gdf_near["geometry"]])
        coordinates = np.array([(point.x, point.y) for point in gdf["geometry"]])
        scaled_coordinates = scaler.fit_transform(coordinates)

        db = DBSCAN(eps=eps, min_samples=3, metric="haversine").fit(np.radians(scaled_coordinates))
        gdf["cluster"] = db.labels_

    # sep clusters into separate GeoDataFrames
    gdf_clusters = [gdf[gdf["cluster"] == i] for i in range(max(gdf["cluster"]) + 1)]

    # Polygonize
    gdf_polygons = [
        alphashape.alphashape(
            points=[(point.x, point.y) for point in cluster["geometry"]], alpha=alpha
        )
        for cluster in gdf_clusters
    ]

    return gdf_polygons


def points_inside(gdf: GeoDataFrame, bbox: Polygon) -> List[Point]:
    """
    Returns all points in gdf that are inside bbox.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of points.
        - bbox (Polygon): A Polygon which defines the region to be queried.

    Returns:
        - matches (List[Point]): A list of all points in gdf that are inside bbox.
    """
    # return all points in gdf that are inside bbox
    matches_index = list(gdf.sindex.query(geometry=bbox))
    matches = gdf.iloc[matches_index]

    return matches["geometry"]


def spatial_filter(gdf: GeoDataFrame, geometry) -> GeoDataFrame:
    """
    Returns a GeoDataFrame with all points inside `geometry` removed.

    Args:
        - gdf (GeoDataFrame): A GeoDataFrame containing a set of points.
        - geometry : shapely.Geometry or array-like of geometries
          (numpy.ndarray, GeoSeries, GeometryArray). From Geopandas documentation.

    Returns:
        - gdf_filtered (GeoDataFrame): A GeoDataFrame with all points inside `geometry` removed.
    """
    return gdf.drop(list(gdf.sindex.query(geometry=geometry)))


def points_to_chunked_array(
    lat_range: tuple, lon_range: tuple, gdf: GeoDataFrame, grid_size: float = 1
) -> List[np.ndarray]:
    """
    Creates a ragged array of ndarrays containing [lon,lat] points that are
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

    Returns:
        - pts_array_list (List[np.ndarray]): A list of ndarrays containing [lon,lat] points that
          are geographically located inside a corresponding grid cell.
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
        range(len(grid_pts) - 1), desc="Grouping bathymetric points by grid cell."
    ):  # lats
        for j in range(len(grid_pts[0]) - 1):  # lons
            # (lon1, lat1, lon2, lat2)
            bbox = box(
                grid_pts[i][j][1],
                grid_pts[i][j][0],
                grid_pts[i + 1][j + 1][1],
                grid_pts[i + 1][j + 1][0],
            )
            pts_array_list.append(
                np.array([[point.x, point.y] for point in points_inside(gdf, bbox)])
            )

    return pts_array_list


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

    for index, row in unclustered_points.iterrows():

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


if __name__ == "__main__":
    main()
