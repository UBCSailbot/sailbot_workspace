#include <iostream>
#include <string>

#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/sensors.pb.h"
#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/waypoint.pb.h"

int main(int argc, char * argv[])
{
    if (argc > 1 && std::string(argv[1]) == "decode") {
        std::string in((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());

        Polaris::Sensors::Path path;
        if (!path.ParseFromString(in)) {
            std::cerr << "Failed to parse protobuf message" << std::endl;
            return 1;
        }

        for (int i = 0; i < path.waypoints_size(); i++) {
            const auto & wp = path.waypoints(i);
            std::cout << wp.latitude() << " " << wp.longitude() << "\n";
        }

        return 0;
    }

    // Default: encode mode
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
