#include <iostream>
#include <string>

#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/sensors.pb.h"
#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/waypoint.pb.h"

// This file is intended to create protobuf objects from given global path objects f
// or use with satellite_test_utils.py

int main()
{
    Polaris::Sensors::Path path;

    double lat, lon;
    while (std::cin >> lat >> lon) {
        auto * wp = path.add_waypoints();
        wp->set_latitude(lat);
        wp->set_longitude(lon);
    }

    std::string out;
    path.SerializeToString(&out);

    std::cout.write(out.data(), out.size());
    return 0;
}


