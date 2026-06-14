import argparse
import os
import pickle
from typing import Any

import geopandas as gpd
import numpy as np
import pyproj
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

WGS84 = pyproj.CRS("EPSG:4326")
MERCATOR = pyproj.CRS("EPSG:3857")
# UTM zone 10N — a metre-based CRS over southern BC, used for accurate metre offsets.
UTM_10N = pyproj.CRS("EPSG:26910")

# Paths are anchored to this file so the script works regardless of the current directory.
LAND_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_DIR = os.path.join(LAND_DIR, "shp")
PKL_PATH = os.path.join(LAND_DIR, "pkl", "land.pkl")

# Land sources: the full OSM Pacific coastline for production, and the Vancouver
# neighbourhood boundaries used as mock land for on-water test plans.
SHP_SOURCES = {
    "production": "complete_land_data.shp",
    "on_water": "local-area-boundary.shp",
}


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


def extrude_edge_to_point(
    polygon: Polygon,
    target_lat: float,
    target_lon: float,
    section_width_m: float = 300.0,
    reach_margin_m: float = 150.0,
    working_crs: pyproj.CRS = UTM_10N,
) -> Polygon:
    """Locally extrude a polygon's nearest edge outward to reach a target point.

    Used to push a coastline land mass back out to a real feature (e.g. a Jericho Pier) over a
    limited section, after a uniform reduction pulled the edge too far inland. The added
    section is kept aligned with the coast: its outer edge is parallel to the adjacent
    coastline, and its side walls are perpendicular to it, so the result follows the local
    shoreline orientation rather than the lat/lon axes.

    The operation is near-idempotent: if the target point is already on the polygon's edge,
    effectively no area is added.

    Args:
        polygon (Polygon): Land polygon in WGS84 (EPSG:4326) lat/lon.
        target_lat (float): Latitude of the point to extrude out to.
        target_lon (float): Longitude of the point to extrude out to.
        section_width_m (float): Width of the extruded section along the coast, in metres.
        reach_margin_m (float): Extra inland depth (metres) beyond the target so the added
            wedge reliably overlaps the existing polygon.
        working_crs (pyproj.CRS): Metre-based CRS used for the offset maths.

    Returns:
        Polygon: The extruded land polygon in WGS84 (EPSG:4326).
    """
    poly_m = gpd.GeoSeries([polygon], crs=WGS84).to_crs(working_crs).iloc[0]
    if poly_m.geom_type == "MultiPolygon":
        poly_m = max(poly_m.geoms, key=lambda g: g.area)
    point_m = gpd.GeoSeries([Point(target_lon, target_lat)], crs=WGS84).to_crs(working_crs).iloc[0]
    P = np.array([point_m.x, point_m.y])

    # Estimate the local coastline direction by densifying the edge and fitting a line to the
    # vertices within one section width of the target point.
    ring = LineString(poly_m.exterior.coords)
    samples = np.array([ring.interpolate(s).coords[0] for s in np.arange(0, ring.length, 3.0)])
    near = samples[np.hypot(samples[:, 0] - P[0], samples[:, 1] - P[1]) < section_width_m]
    if len(near) < 2:
        raise ValueError("no polygon edge found near the target point")
    slope = np.polyfit(near[:, 0], near[:, 1], 1)[0]
    along = np.array([1.0, slope])
    along /= np.linalg.norm(along)
    across = np.array([-along[1], along[0]])
    centroid = np.array([poly_m.centroid.x, poly_m.centroid.y])
    if np.dot(P - centroid, across) < 0:
        across = -across  # orient the across-coast axis from the polygon toward the target

    # Coast-aligned slab capped by the line through the target parallel to the coast.
    depth = poly_m.boundary.distance(point_m) + reach_margin_m
    half = section_width_m / 2.0
    cap = Polygon(
        [
            P + a * along + b * across
            for a, b in [(-half, -depth), (half, -depth), (half, 0.0), (-half, 0.0)]
        ]
    )
    wedge = cap.difference(poly_m).intersection(poly_m.buffer(depth))
    result_m = unary_union([poly_m, wedge]).buffer(0)

    result = gpd.GeoSeries([result_m], crs=working_crs).to_crs(WGS84).iloc[0]
    return make_valid(result).buffer(0)


def pickle_land(source: str = "production"):
    """Generates a land dataset and stores it in PKL format for long term storage on disk.

    Land data is saved to pkl/land.pkl.

    Args:
        source (str): Which land source to pickle. "production" uses the full OSM Pacific
            coastline; "test" uses the Vancouver neighbourhood boundaries (mock land for
            on-water test plans).

    Raises:
        ValueError: If source is not a known land source.
    """
    if source not in SHP_SOURCES:
        raise ValueError(f"unknown source {source!r}; expected one of {sorted(SHP_SOURCES)}")

    gdf = gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[source]))

    # Dissolve all polygons into one geometry and repair any invalid/overlapping rings.
    land = unary_union(list(gdf["geometry"])).buffer(0)

    # Downstream code expects a MultiPolygon, even if the union collapses to a single Polygon.
    if land.geom_type == "Polygon":
        land = MultiPolygon([land])

    dump_pkl(land, PKL_PATH)
    print("Land data pickled to", PKL_PATH + f" (land source: {source})")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate land.pkl from a land shapefile source.")
    parser.add_argument(
        "--source",
        choices=sorted(SHP_SOURCES),
        default="production",
        help="Land source to pickle (default: production).",
    )
    parser.add_argument(
        "--extrude",
        action="store_true",
        help="Whether to extrude the land out to the Jericho Pier for better on-water test plan coverage.",
    )

    args = parser.parse_args()
    if args.extrude:
        # Extrude the land out to the Jericho Pier (target lat/lon)
        # for better on-water test plan coverage.
        # TODO: Incomplete
        extrude_edge_to_point(
            polygon=gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[args.source])).geometry.iloc[
                0
            ],
            target_lat=49.2885,
            target_lon=-123.1610,
        )
    else:
        pickle_land(args.source)
