# Remote Server Standalone

This folder contains a standalone version of the remote_transceiver server and sailbot_db, decoupled from ROS. It is ready to be copied into a new repository and built independently.

## Structure

- `src/` - Source files
- `inc/` - Header files
- `lib/` - Third-party or static libraries (if needed)
- `scripts/` - Helper scripts

## Build

Use the provided CMakeLists.txt to build the project. All ROS dependencies have been removed.

Note that a clean build (removing all dependencies etc.) takes a long time and should be avoided. There's a chance the script won't fetch all packages required and manual installations and setup may be required, especially for the MongoDB CXX driver.

Just rebuilding any `.cpp` or `.h` file modifications by running `build.sh` should suffice for most cases. This also takes a few minutes, but you should see progress % updates every minute or so.
