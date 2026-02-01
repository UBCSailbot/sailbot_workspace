#!/bin/bash

# This script creates a virtual environment specifically for use in Jupyter notebooks to ensure
# compatibility between installations of matplotlib, pandas, and numpy and to keep those installations
# separate from our base system
set -e

FLAG="$HOME/ros/notebook_env/.setup_done"
if [ -f "$FLAG" ]; then
    echo "Notebook environment already set up, skipping."
    exit 0
fi

sudo DEBIAN_FRONTEND=noninteractive apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3.10-venv python3-pip
python3.10 -m venv ~/ros/notebook_env --system-site-packages
source ~/ros/notebook_env/bin/activate
pip3 install --upgrade matplotlib pandas numpy==1.25 numexpr==2.8.4 bottleneck==1.3.6
python -m ipykernel install --user --name=notebook_env --display-name "Python (notebook_env)"
sudo DEBIAN_FRONTEND=noninteractive apt-get remove -y python3-matplotlib

touch "$FLAG"
