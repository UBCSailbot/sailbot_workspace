# Land Data

The land data used for this project is Open Street Maps data downloaded from here: <https://osmdata.openstreetmap.de/data/land-polygons.html>

The specific dataset used is the one described with "Large polygons are split, use for larger zoom levels"

This data may be used for ANY purpose under the Open Database License (ODbL) v1.0. <https://www.openstreetmap.org/copyright>

The ./shp/ directory contains the shapefiles for a subset of the data from the data file downloaded from the source above. The ./shp/ data files contain only land polygons close to the shores around the pacific ocean are included, to cut down on file size.

The data can be explored using Geopandas, ex:
`gdf = gpd.read_file("shp/complete_land_data.shp")`

The same data stored in ./shp/ is also stored as a single Shapely Multipolygon object that is encoded into the ./pkl/land.pkl file for long term storage and easy loading into our pathfinding program during runtime.
