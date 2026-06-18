import argparse
import os
import pickle
from typing import Any

import geopandas as gpd
import matplotlib
import numpy as np
import pyproj
import yaml
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union
from shapely.validation import make_valid

matplotlib.use("Agg")

WGS84 = pyproj.CRS("EPSG:4326")
UTM_10N = pyproj.CRS("EPSG:26910")  # metre-based CRS over southern BC, for accurate offsets

# Paths are anchored to this file so the script runs regardless of the current directory.
LAND_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_DIR = os.path.join(LAND_DIR, "shp")
PKL_PATH = os.path.join(LAND_DIR, "pkl", "land.pkl")
TEST_PLAN_PATH = os.path.join(LAND_DIR, "..", "test_plans", "jericho_on_water_test.yaml")
CUT_PLOT_PATH = os.path.join(LAND_DIR, "west_point_grey_pier_parallel.png")

# Full OSM Pacific coastline for open ocean scenarios; the original (full-buffer) Vancouver
# neighbourhood boundaries for on-water scenarios. The on-water buffer is kept as is and only
# cut back locally at the pier (see cut_edge_to_point).
SHP_SOURCES = {
    "offshore": "complete_land_data.shp",
    "on_water": "local-area-boundary_backup.shp",
}

JERICHO_PIER = (49.277065, -123.201474)  # (lat, lon) pier the on-water edge is cut back to


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


def cut_edge_to_point(
    polygon: Polygon,
    target_lat: float,
    target_lon: float,
    cut_width_m: float = 1000.0,
    working_crs: pyproj.CRS = UTM_10N,
) -> Polygon:
    """Cut a local notch into a polygon's edge so the edge passes through a target point.

    Keeps the polygon's original outline (its full buffer) everywhere except a short stretch of
    coast around the target, where the land lying seaward of the target is removed so the edge
    is pulled back to it (e.g. the Jericho Pier, used as a visible on-water reference). The
    notch spans ``cut_width_m`` along the coast; its two side walls are intentional
    discontinuities with the unchanged neighbouring edge.

    The operation is idempotent when no land lies seaward of the target within the window.

    Args:
        polygon (Polygon): Land polygon in WGS84 (EPSG:4326) lat/lon.
        target_lat (float): Latitude of the point to cut the edge back to.
        target_lon (float): Longitude of the point to cut the edge back to.
        cut_width_m (float): Width (metres) of the cut section of coast, centred on the target.
        working_crs (pyproj.CRS): Metre-based CRS used for the offset maths.

    Returns:
        Polygon: The cut land polygon in WGS84 (EPSG:4326).
    """
    poly_m = gpd.GeoSeries([polygon], crs=WGS84).to_crs(working_crs).iloc[0]
    if poly_m.geom_type == "MultiPolygon":
        poly_m = max(poly_m.geoms, key=lambda g: g.area)

    point_m = gpd.GeoSeries([Point(target_lon, target_lat)], crs=WGS84).to_crs(working_crs).iloc[0]
    P = np.array([point_m.x, point_m.y])

    # Local coast direction = the exterior segment nearest the target; the normal points seaward.
    coords = np.array(poly_m.exterior.coords)
    i = min(
        range(len(coords) - 1),
        key=lambda i: LineString([coords[i], coords[i + 1]]).distance(point_m),
    )
    tangent = (coords[i + 1] - coords[i]) / np.linalg.norm(coords[i + 1] - coords[i])
    normal = np.array([-tangent[1], tangent[0]])
    centroid = np.array([poly_m.centroid.x, poly_m.centroid.y])
    if np.dot(coords[i] - centroid, normal) < 0:
        normal = -normal  # orient seaward, away from the polygon interior

    # Cutter: a box covering the land seaward of the line through the target (parallel to the
    # coast), limited to a cut_width_m window along the coast. Subtracting it pulls the edge
    # back to the target locally and leaves the rest of the outline untouched.
    half = cut_width_m / 2.0
    reach = max(0.0, float(((coords - P) @ normal).max())) + 1.0
    cutter = Polygon(
        [
            P + a * tangent + b * normal
            for a, b in [(-half, 0.0), (half, 0.0), (half, reach), (-half, reach)]
        ]
    )
    result_m = poly_m.difference(cutter).buffer(0)
    if result_m.geom_type == "MultiPolygon":
        result_m = max(result_m.geoms, key=lambda g: g.area)

    result = gpd.GeoSeries([result_m], crs=working_crs).to_crs(WGS84).iloc[0]
    return make_valid(result).buffer(0)


def pickle_land(source: str = "offshore", cut_to: tuple[float, float] | None = None):
    """Generates a land dataset and stores it in PKL format for long term storage on disk.

    Land data is saved to pkl/land.pkl.

    Args:
        source (str): Which land source to pickle. "offshore" uses the full OSM Pacific
            coastline; "on_water" uses the Vancouver neighbourhood boundaries (mock land for
            on-water test plans).
        cut_to (tuple[float, float] | None): (lat, lon) point to cut the nearest land
            mass back to (e.g. the Jericho Pier) before pickling. Only the polygon whose edge is
            closest to the point is cut; all other land is left unchanged. If None, no cut is
            applied.

    Raises:
        ValueError: If source is not a known land source.
    """
    if source not in SHP_SOURCES:
        raise ValueError(f"unknown source {source!r}; expected one of {sorted(SHP_SOURCES)}")

    gdf = gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[source]))
    polygons = list(gdf["geometry"])

    if cut_to is not None:
        target_lat, target_lon = cut_to
        target = Point(target_lon, target_lat)
        nearest = min(range(len(polygons)), key=lambda i: polygons[i].distance(target))
        polygons[nearest] = cut_edge_to_point(polygons[nearest], target_lat, target_lon)

    land = unary_union(polygons).buffer(0)

    # Downstream code expects a MultiPolygon, even when the union collapses to one Polygon.
    if land.geom_type == "Polygon":
        land = MultiPolygon([land])

    dump_pkl(land, PKL_PATH)
    print("Land data pickled to", PKL_PATH + f" (land source: {source})")


def plot_cut(source: str, target: tuple, out_path: str = CUT_PLOT_PATH):
    """Render a verification figure of the cut result and save it to out_path.

    Replays the cut exactly as pickle_land applies it (the polygon nearest the target is cut),
    then plots the cut land against the original (uncut) coastline and the on-water test track
    so the result can be eyeballed.

    Args:
        source (str): Land source to plot (see SHP_SOURCES).
        target (tuple[float, float]): (lat, lon) point the edge was cut back to.
        out_path (str): Where to write the PNG.
    """
    target_lat, target_lon = target
    point = Point(target_lon, target_lat)

    gdf = gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[source]))
    polygons = list(gdf["geometry"])
    nearest = min(range(len(polygons)), key=lambda i: polygons[i].distance(point))
    name = gdf.iloc[nearest]["Name"]
    original = polygons[nearest]
    cut = cut_edge_to_point(original, target_lat, target_lon)
    removed = make_valid(original.difference(cut))

    with open(TEST_PLAN_PATH) as f:
        waypoints = yaml.safe_load(f)["global_path"]["waypoints"]
    track_lon = [w["longitude"] for w in waypoints]
    track_lat = [w["latitude"] for w in waypoints]

    fig, ax = plt.subplots(figsize=(11, 8))
    gpd.GeoSeries([cut]).plot(ax=ax, facecolor="#cdebc5", edgecolor="#2c7a2c", lw=2, zorder=2)
    gpd.GeoSeries([removed]).plot(
        ax=ax, facecolor="#ff7f0e", alpha=0.55, edgecolor="#d35400", hatch="///", zorder=3
    )
    gpd.GeoSeries([original.boundary]).plot(ax=ax, color="#c0392b", lw=1.6, ls="--", zorder=4)
    ax.plot(track_lon, track_lat, "-", color="#1f4e8c", lw=1.3, zorder=5)
    ax.scatter(track_lon, track_lat, s=8, color="#1f4e8c", zorder=6)
    ax.scatter(target_lon, target_lat, s=180, marker="*", color="black", zorder=7)
    ax.annotate(
        "pier reference",
        (target_lon, target_lat),
        textcoords="offset points",
        xytext=(10, -20),
        fontsize=9,
        fontweight="bold",
    )
    ax.set_xlim(target_lon - 0.006, target_lon + 0.006)
    ax.set_ylim(target_lat - 0.004, target_lat + 0.0035)
    ax.set_aspect(1 / 0.652)  # rough lon/lat aspect at this latitude so distances look right
    ax.legend(
        handles=[
            Patch(facecolor="#cdebc5", edgecolor="#2c7a2c", label="Cut land (--cut)"),
            Patch(facecolor="#ff7f0e", alpha=0.55, hatch="///", label="Removed (cut to pier)"),
            Line2D([0], [0], color="#c0392b", ls="--", label="Original edge (kept buffer)"),
            Line2D([0], [0], color="#1f4e8c", marker="o", ms=4, label="On-water test track"),
            Line2D([0], [0], color="black", marker="*", ls="", ms=12, label="Pier reference"),
        ],
        loc="lower right",
        fontsize=9,
        framealpha=0.95,
    )
    ax.set_title(f"{name} - edge cut to the pier by pickle_land --cut", fontsize=12)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.grid(True, ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print("Cut verification plot saved to", out_path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate land.pkl from a land shapefile source.")
    parser.add_argument(
        "--source",
        choices=sorted(SHP_SOURCES),
        default="offshore",
        help="Land source to pickle (default: offshore).",
    )
    parser.add_argument(
        "--cut",
        action="store_true",
        help="Cut the on-water land back to a reference (Jericho Pier) for on-water testing.",
    )

    args = parser.parse_args()

    if args.cut:
        pickle_land(args.source, cut_to=JERICHO_PIER)
        plot_cut(args.source, JERICHO_PIER)
    else:
        pickle_land(args.source, cut_to=None)
