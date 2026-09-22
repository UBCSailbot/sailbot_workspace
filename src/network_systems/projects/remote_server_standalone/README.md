# Remote Server Standalone

This folder contains a standalone version of the remote_transceiver server and sailbot_db,
decoupled from ROS. It is ready to be copied into a new repository and built independently.

## Structure

- `src/` - Source files and named C++ module interfaces
- `inc/` - Protobuf-generated headers
- `lib/` - Third-party or static libraries (if needed)
- `scripts/` - Helper scripts

## Build

Use the provided CMakeLists.txt to build the project. All ROS dependencies
have been removed. The build requires Clang 16+, CMake 3.28+, and Ninja 1.11+
for C++20 module dependency scanning. The public modules are
`network_systems.remote.constants`, `network_systems.remote.database`, and
`network_systems.remote.transceiver`.

Remove the existing `build/` directory once when switching from the previous
Makefiles build to Ninja. After that migration, clean builds take a long time
and should be avoided. There's a chance the script won't fetch all packages
required and manual installations and setup may be required, especially for
the MongoDB CXX driver.

Just rebuilding any `.cpp` or `.cppm` file modifications by running `build.sh`
should suffice for most cases. This also takes a few minutes, but you should
see progress % updates every minute or so.
