# GPS data of the simulated with respect to the Earth (in the global reference frame).
GPS global_gps

# Contains both position and orientation (in quaternions, following a East-North-Up convention)
# of the simulated boat with respect to the Earth (in the global reference frame).
# Position Unit: km, distance from the origin
geometry_msgs/Pose global_pose

# Direction that the wind is going TOWARDS. Note that this is difference from the WindSensor type
# in custom_interfaces, where WindSensor.direction gives us the direction where the wind is coming
# FROM (basically a 180 degree phase shift in the velocity direction).
#
# Contains the linear component of the velocity.
# Linear Velocity Unit: km/hr
geometry_msgs/Vector3 wind_velocity

# Direction that the current is going TOWARDS.
#
# Contains the linear component of the velocity.
# Linear Velocity Unit: km/hr
geometry_msgs/Vector3 current_velocity

# Timestamp. Contains time since start in seconds and the frame number.
std_msgs/Header header
