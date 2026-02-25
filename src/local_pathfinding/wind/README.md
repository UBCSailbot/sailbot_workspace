# Wind

This directory is for any research or calculations related to our velocity prediction model for Sailbot.

When evaluating potential paths for Sailbot to take, we take into account how fast we expect Sailbot to travel along a given path for a given true wind speed and direction. To make this prediction we need to come up with a lookup table of estimated boat speeds for certain true wind speeds and points of sail. The Jupyter notebook in this directory does exactly that.

The source data used by the notebook file is ./polar_sailing_speed_prediction_data and it was downloaded from <https://jieter.github.io/orc-data/site/#GRE/GRE1772>.

While it is data from a different vessel, the data is scaled to match the estimated max speed of Sailbot while still maintaining the shape you would expect to see in a sailboat's polar plot.
