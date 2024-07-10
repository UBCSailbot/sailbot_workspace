# Integration Testing 

ais\_file - sets AIS boat positions, speeds, and directions

gps\_file - sets sailbot's initial position latlon

goal\_file - sets the goal latlon (note: this is the final goal, not necessarily the first waypoint of the global path if it is sufficiently far away from the sailbot start

wind\_file - sets the initial wind speed and direction

## How to run an integration test:

* Source the workspace
* Navigate to a json test directory eg: `cd json/obstacles-currently-btwn-moving-away_goal-south_wind-east`
* Run pathfinding with the json files in this directory: `roslaunch local_pathfinding all.launch ais_file:=$(pwd)/myAIS.json gps_file:=$(pwd)/myGPS.json goal_file:=$(pwd)/myGoal.json wind_file:=$(pwd)/myWind.json`


Each test directory name describes the obstacles, goal, and wind setup for each test. 

## How to create your own files

You can create your own unit test files by copying the json files here, or by starting a simulation and then running (in another terminal) `rosrun local_pathfinding json_dumper.py`, which will save the current AIS and GPS conditions into files in your current directory.
