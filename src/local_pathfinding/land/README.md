<!-- markdownlint-disable MD013 -->
# Land Data

This directory generates the land mass that pathfinding (OMPL) treats as off-limits. Raw
shapefiles in `./shp/` are dissolved into a single Shapely `MultiPolygon` and saved to
`./pkl/land.pkl` for fast loading at runtime.

## Sources

There are two land sources, selected with `--source`:

- **`offshore`** — OpenStreetMap land polygons (the "Large polygons are split, use for larger
  zoom levels" dataset) from <https://osmdata.openstreetmap.de/data/land-polygons.html>. Used
  for launch. `./shp/` keeps only the polygons near the Pacific shore to cut file size.
  Licensed under the [Open Database License (ODbL) v1.0](https://www.openstreetmap.org/copyright).
- **`on_water`** — Vancouver [local-area boundaries](https://opendata.vancouver.ca/explore/dataset/local-area-boundary/information/?disjunctive.name).
  Used for on-water testing at Jericho Beach, because the OSM data has kilometre-scale
  resolution where we need metre-scale.

Explore any shapefile with GeoPandas: `gdf = gpd.read_file("shp/complete_land_data.shp")`.

## Why the cut

The `on_water` source keeps West Point Grey's original coastline, whose edge extends a buffer
out past the true shoreline. That buffer is deliberate: the water near the coast is shallow, so
a polygon flush with the shore would let Polaris's keel ground on the ocean floor. But the crew
still needs a clear, visible land/water boundary to judge position from the boat.

So `pickle_land --cut` carves a local notch at **Jericho Pier** — a visible, fixed reference —
pulling the edge back through the pier over a short section of coast. Everywhere else the
original buffer is kept; only the stretch of coast around the pier is cut. The discontinuity
where the notch meets the neighbouring edge is expected and harmless.

![West Point Grey edge cut back to Jericho Pier](west_point_grey_pier_parallel.png)

Running with `--cut` also writes this verification figure so the result can be eyeballed: green
is the kept land, the orange hatch is the notch removed at the pier, the dashed red line is the
original (kept) buffer edge, and the star is the pier.

## How to run

```sh
# Before launch (full offshore coastline):
python3 src/local_pathfinding/land/pickle_land_data.py --source offshore

# Before on-water testing at Jericho Beach (cut to the pier reference):
python3 src/local_pathfinding/land/pickle_land_data.py --source on_water --cut
```

Each run overwrites `./pkl/land.pkl`.
