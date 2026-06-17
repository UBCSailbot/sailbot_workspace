import argparse
import os
import pickle
from typing import Any, Optional

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
MERCATOR = pyproj.CRS("EPSG:3857")
# UTM zone 10N — a metre-based CRS over southern BC, used for accurate metre offsets.
UTM_10N = pyproj.CRS("EPSG:26910")

# Paths are anchored to this file so the script works regardless of the current directory.
LAND_DIR = os.path.dirname(os.path.abspath(__file__))
SHP_DIR = os.path.join(LAND_DIR, "shp")
PKL_PATH = os.path.join(LAND_DIR, "pkl", "land.pkl")

# Pristine (pre-edit) on-water boundaries and the test plan used as overlays in the
# extrusion verification plot.
PRISTINE_SHP = os.path.join(SHP_DIR, "local-area-boundary_backup.shp")
TEST_PLAN_PATH = os.path.join(LAND_DIR, "..", "test_plans", "jericho_on_water_test.yaml")
EXTRUSION_PLOT_PATH = os.path.join(LAND_DIR, "west_point_grey_pier_parallel.png")

# Land sources: the full OSM Pacific coastline for production, and the Vancouver
# neighbourhood boundaries used as mock land for on-water test plans.
SHP_SOURCES = {
    "production": "complete_land_data.shp",
    "on_water": "local-area-boundary.shp",
}

# Jericho Pier (lat, lon) — extrude the nearest land mass out to here so on-water test plans
# have land coverage up to the pier.
JERICHO_PIER = (49.277065, -123.201474)


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


def pickle_land(source: str = "production", extrude_to: Optional[tuple] = None):
    """Generates a land dataset and stores it in PKL format for long term storage on disk.

    Land data is saved to pkl/land.pkl.

    Args:
        source (str): Which land source to pickle. "production" uses the full OSM Pacific
            coastline; "on_water" uses the Vancouver neighbourhood boundaries (mock land for
            on-water test plans).
        extrude_to (tuple[float, float] | None): Optional (lat, lon) point to extrude the
            nearest land mass out to (e.g. the Jericho Pier) before pickling. Only the polygon
            whose edge is closest to the point is extruded; all other land is left unchanged
            and added back in by the union. If None, no extrusion is applied.

    Raises:
        ValueError: If source is not a known land source.
    """
    if source not in SHP_SOURCES:
        raise ValueError(f"unknown source {source!r}; expected one of {sorted(SHP_SOURCES)}")

    gdf = gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[source]))
    polygons = list(gdf["geometry"])

    if extrude_to is not None:
        target_lat, target_lon = extrude_to
        target = Point(target_lon, target_lat)
        # Extrude only the land mass nearest the target; the rest is restored by the union below.
        nearest = min(range(len(polygons)), key=lambda i: polygons[i].distance(target))
        polygons[nearest] = extrude_edge_to_point(polygons[nearest], target_lat, target_lon)

    # Dissolve all polygons into one geometry and repair any invalid/overlapping rings.
    land = unary_union(polygons).buffer(0)

    # Downstream code expects a MultiPolygon, even if the union collapses to a single Polygon.
    if land.geom_type == "Polygon":
        land = MultiPolygon([land])

    dump_pkl(land, PKL_PATH)
    print("Land data pickled to", PKL_PATH + f" (land source: {source})")


def plot_extrusion(source: str, target: tuple, out_path: str = EXTRUSION_PLOT_PATH):
    """Render a verification figure of the extrusion result and save it to out_path.

    Replays the extrusion exactly as pickle_land applies it (the polygon nearest the target is
    extruded), then plots the extruded land against the pristine original coastline and the
    on-water test track so the result can be eyeballed.

    Requires matplotlib and pyyaml, which are not runtime dependencies of this package, so the
    plot is skipped with a message if either is missing.

    Args:
        source (str): Land source to plot (see SHP_SOURCES).
        target (tuple[float, float]): (lat, lon) point the land was extruded out to.
        out_path (str): Where to write the PNG.
    """

    target_lat, target_lon = target
    point = Point(target_lon, target_lat)

    # Same selection + extrusion that pickle_land performs.
    gdf = gpd.read_file(os.path.join(SHP_DIR, SHP_SOURCES[source]))
    polygons = list(gdf["geometry"])
    nearest = min(range(len(polygons)), key=lambda i: polygons[i].distance(point))
    name = gdf.iloc[nearest]["Name"]
    extruded = extrude_edge_to_point(polygons[nearest], target_lat, target_lon)

    # Pristine original edge of the same land mass, and the land removed relative to it.
    pristine = gpd.read_file(PRISTINE_SHP)
    original = pristine[pristine["Name"] == name].geometry.iloc[0]
    removed = make_valid(original.difference(extruded))

    with open(TEST_PLAN_PATH) as f:
        waypoints = yaml.safe_load(f)["global_path"]["waypoints"]
    track_lon = [w["longitude"] for w in waypoints]
    track_lat = [w["latitude"] for w in waypoints]

    fig, ax = plt.subplots(figsize=(11, 8))
    gpd.GeoSeries([extruded]).plot(ax=ax, facecolor="#cdebc5", edgecolor="#2c7a2c", lw=2, zorder=2)
    gpd.GeoSeries([removed]).plot(
        ax=ax, facecolor="#ff7f0e", alpha=0.55, edgecolor="#d35400", hatch="///", zorder=3
    )
    gpd.GeoSeries([original.boundary]).plot(ax=ax, color="#c0392b", lw=1.6, ls="--", zorder=4)
    ax.plot(track_lon, track_lat, "-", color="#1f4e8c", lw=1.3, zorder=5)
    ax.scatter(track_lon, track_lat, s=8, color="#1f4e8c", zorder=6)
    ax.scatter(target_lon, target_lat, s=180, marker="*", color="black", zorder=7)
    ax.annotate(
        "extrusion target",
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
            Patch(facecolor="#cdebc5", edgecolor="#2c7a2c", label="Extruded land (--extrude)"),
            Patch(
                facecolor="#ff7f0e", alpha=0.55, hatch="///", label="Removed vs pristine (water)"
            ),
            Line2D([0], [0], color="#c0392b", ls="--", label="Pristine original edge"),
            Line2D([0], [0], color="#1f4e8c", marker="o", ms=4, label="On-water test track"),
            Line2D([0], [0], color="black", marker="*", ls="", ms=12, label="Extrusion target"),
        ],
        loc="lower right",
        fontsize=9,
        framealpha=0.95,
    )
    ax.set_title(f"{name} - extrusion produced by pickle_land --extrude", fontsize=12)
    ax.set_xlabel("Longitude")
    ax.set_ylabel("Latitude")
    ax.grid(True, ls=":", alpha=0.4)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print("Extrusion verification plot saved to", out_path)


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
        help="Whether to extrude the land out to a reference for on-water test.",
    )

    args = parser.parse_args()
    pickle_land(args.source, extrude_to=JERICHO_PIER if args.extrude else None)
    if args.extrude:
        # Save a figure of the extrusion result so the output can be verified.
        plot_extrusion(args.source, JERICHO_PIER)
