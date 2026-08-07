Path global_path
Path local_path
# When GPS is unavailable: latitude=91°, longitude=181°, speed=-1 km/h, heading=360°.
GPS gps
# When wind is unavailable: speed=-1 km/h and direction=0°.
WindSensor filtered_wind_sensor
# When AIS is unavailable: ships contains exactly one sentinel ship with id=0.
AISShips ais_ships
# contains Land and Boat obstacles differentiated by obstacle_type attribute
HelperObstacle[] obstacles
# When desired heading is unavailable: heading=360° and sail=false.
DesiredHeading desired_heading
# Latest e-compass boat heading; 360° means unavailable or timed out.
HelperHeading heading
string replan_reason
int32 remaining_waypoints
