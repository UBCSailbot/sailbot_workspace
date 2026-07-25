# Direction the bow of the boat is pointing towards
# Unit: degrees, 0° north, increases CW
# Range: -180 < heading <= 180
# NET stores AIS COG internally in [0, 360), then converts it to this range when publishing.
float32 heading
