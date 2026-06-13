/**
 * This is a helper utility used in the satellite_test_utils.py script.
 * It encodes and decodes protobuf objects.
 * This is primarily used for manual testing of satellite data transfer.
 */
#include <fstream>
#include <iostream>
#include <string>

#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/global_path.pb.h"
#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/sensors.pb.h"
#include "/workspaces/sailbot_workspace/build/network_systems/lib/protofiles/waypoint.pb.h"

#define htons(x) __bswap_16(x)

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
    Polaris::GlobalPath path;

    float lat;
    float lon;
    while (std::cin >> lat >> lon) {
        auto * wp = path.add_waypoints();
        wp->set_latitude(lat);
        wp->set_longitude(lon);
    }

    std::string out;
    path.SerializeToString(&out);
    uint16_t    size = static_cast<uint16_t>(out.size());
    uint16_t    be   = htons(size);
    std::string size_prefix(reinterpret_cast<const char *>(&be), sizeof(be));

    std::string raw_bytes = size_prefix + out;

    std::ofstream outfile("./serialized_data.bin", std::ios::binary);
    outfile.write(size_prefix.data(), size_prefix.size());  //NOLINT
    outfile.write(out.data(), static_cast<std::streamsize>(out.size()));
    outfile.close();

    std::cout.write(raw_bytes.data(), raw_bytes.size());
    return 0;
}
