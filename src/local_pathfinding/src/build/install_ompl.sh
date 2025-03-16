#!/bin/bash
set -e

arch=$(uname -m)
AARCH_64_WHL_URL="https://github.com/ompl/ompl/releases/download/prerelease/ompl-1.6.0-cp310-cp310-manylinux_2_28_aarch64.whl"
AARCH_64_WHL_FILE="/tmp/ompl-1.6.0-cp310-cp310-manylinux_2_28_aarch64.whl"
X86_64_WHL_URL="https://github.com/ompl/ompl/releases/download/prerelease/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"
X86_64_WHL_FILE="/tmp/ompl-1.6.0-cp310-cp310-manylinux_2_28_x86_64.whl"

if [ "$arch" = "x86_64" ]; then
    echo "running on x86_64 architecture."
    curl -L -o "$X86_64_WHL_FILE" "$X86_64_WHL_URL"
    pip3 install "$X86_64_WHL_FILE"
    rm -f "$X86_64_WHL_FILE"
    # sudo ln -sf /path/to/ompl/installation /workspaces/sailbot_workspace/install/lib/python3.10/site-packages
    # sudo ln -sf /path/to/ompl/installation /usr/lib/python3/dist-packages
elif [ "$arch" = "aarch64" ]; then
    echo "running on aarch64 architecture."
    curl -L -o "$AARCH_64_WHL_FILE" "$AARCH_64_WHL_URL"
    pip3 install "$AARCH_64_WHL_FILE"
    rm -f "$AARCH_64_WHL_FILE"
    # sudo ln -sf /path/to/ompl/installation /workspaces/sailbot_workspace/install/lib/python3.10/site-packages
    # sudo ln -sf /path/to/ompl/installation /usr/lib/python3/dist-packages
else
    echo "Unknown architecture: $arch"
fi

if pip3 show ompl 2>&1 | grep -q "not found"; then
    echo "ompl installation failed"
fi
