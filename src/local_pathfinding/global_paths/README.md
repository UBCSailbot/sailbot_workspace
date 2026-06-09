# Global Paths Directory

This directory contains all mock global path CSV files,
as well as a path_builder.py module.

## CSV Format and Waypoint Ordering

A global path CSV has a `latitude,longitude` header followed by one waypoint
per row. **Waypoints must be ordered final-destination-first**: the first data
row becomes `waypoints[0]`, which the navigate node treats as the **final
destination**, while the **last** data row is the waypoint the boat heads to
**first** after launch. In other words, list the waypoints in the reverse of
the order the boat sails them (destination first, launch area last). This is
the same convention used by the development test plans.

## path_builder.py Module

The path_builder.py module can be used two different ways:

GUI: The module serves a basic flask application which allows
   for easy creation and modification of mock global paths.
   To launch the GUI, run the following command from the root of the
   local_pathfinding repository:

`python3 global_paths/path_builder/path_builder.py`

CLI: The module can also be run from the command line for faster and more
performant operations.
To use, run the following command(s) from the root of the local_pathfinding
repository:

<!-- markdownlint-disable-next-line MD013 -->
`python3 global_paths/path_builder/path_builder.py --file_path <path_to_csv_file>` (to plot)

<!-- markdownlint-disable-next-line MD013 -->
`python3 global_paths/path_builder/path_builder.py --file_path <path_to_csv_file> --interpolate 30.0` (to interpolate)

<!-- markdownlint-disable-next-line MD013 -->
`python3 global_paths/path_builder/path_builder.py --delete` (to delete all generated paths with timestamps)

Using any of the command line arguments will keep the GUI from launching.

### GUI Usage

The Path Builder supports the following operations:

- Path creation and editing on a 2D OpenStreetMap Map
- Path exporting
- Path importing
- Interpolation between waypoints
- Clear path function
- Purge generated path files
- 3D visualization of path

Note: Selecting a small value for the interpolation interval spacing or having
a large number of waypoints may cause
performance issues in the GUI as the javascript is not optimized. If this is
an issue,
try increasing the spacing or running the interpolation from the CLI instead.
