# Notebooks

UBC Sailbot's Jupyter notebooks for researching and exporing implementations.

## Setup

To get started using notebooks in the devcontainer:

1. run `install_notebook_env.sh`
2. ctl + shift + p  `Developer: Reload Window`
3. open a `.ipynb` file and select the kernel: _Python (notebook_env)_. If the kernel env does not appear, try clicking refresh or keep running Developer: Reload Window until it does.

## Standards

1. Notebooks should be organized into directories named like the UBC Sailbot repositories they correspond to

# Introducing rosbags

- Rosbags are sqlite db files which contain messages published to a topic that are being recorded. These are not natively supported.
- By converting these db files into a DataFrame, we can use data to create plots and tables.
- To start, begin by executing the code cells in `explore_bag_data.ipynb` in order. You can see the DataFrame by running the `df.head()` command in the third cell. These are the messages that are being published to local_path by gps, wind_sensors, etc.
- By running the 4th code cell, you extract the data from each message into numerical values that are being published to the topic.
