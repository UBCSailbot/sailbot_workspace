#!/bin/bash

set -e

# this will detect the underlying architecture and generate the appropriate bindings (for x86 or aarch64)
g++ -O3 -Wall -shared -std=c++11 -fPIC `python3 -m pybind11 --includes` ompl_bindings.cpp -o pyompl`python3-config --extension-suffix` -I/usr/include/ompl-1.6 -I/usr/include/eigen3 -L/usr/share -lompl

# we need to create symbolic links to the python bindings so that they can be imported during tests and when ROS is running
arch=$(uname -m)

if [ "$arch" = "x86_64" ]; then
    echo "running on x86_64 architecture."
    sudo ln -sf /workspaces/sailbot_workspace/src/local_pathfinding/src/build/pyompl.cpython-310-x86_64-linux-gnu.so /workspaces/sailbot_workspace/install/lib/python3.10/site-packages
    sudo ln -sf /workspaces/sailbot_workspace/src/local_pathfinding/src/build/pyompl.cpython-310-x86_64-linux-gnu.so /usr/lib/python3/dist-packages
elif [ "$arch" = "aarch64" ]; then
    echo "running on aarch64 architecture."
    sudo ln -sf /workspaces/sailbot_workspace/src/local_pathfinding/src/build/pyompl.cpython-310-aarch64-linux-gnu.so /workspaces/sailbot_workspace/install/lib/python3.10/site-packages
    sudo ln -sf /workspaces/sailbot_workspace/src/local_pathfinding/src/build/pyompl.cpython-310-aarch64-linux-gnu.so /usr/lib/python3/dist-packages
else
    echo "Unknown architecture: $arch"
fi

echo "Python bindings for ompl installed successfully"
