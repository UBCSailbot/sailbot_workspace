# Remote Server Standalone

This folder contains a standalone version of the remote_transceiver server and sailbot_db, decoupled from ROS. It is ready to be copied into a new repository and built independently.

## Structure
- `src/` - Source files
- `inc/` - Header files
- `lib/` - Third-party or static libraries (if needed)
- `scripts/` - Helper scripts

## Build
Use the provided CMakeLists.txt to build the project. All ROS dependencies have been removed.
