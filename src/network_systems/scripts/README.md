# Scripts

## Generate ROS Info

```shell
python3 gen_ros_info.py
```

Takes [ros_info.yaml](../ros_info.yaml) and generates a C++ header file and Python file with constants defined within.

## Sailbot DB

```shell
./sailbot_db <db-name> [COMMAND]
./sailbot_db --help
```

Wrapper for the [SailbotDB Utility DB tool](../lib/sailbot_db/src/main.cpp).

- Requires network_systems to be built
- Run with `--help` for full details on how to run
- Can clear, populate, and dump data from a DB

## Rockblock Web Server

This web server is a virtual implementation of the Rockblock Web Servers used to send MT messages. The specification for the RockBlock Web Servers can be found [here](https://docs.groundcontrol.com/iot/rockblock/web-services/sending-mt-message).

This virtual server follows this specification with one notable difference, it will return the specific error code that is contained in the `data` section of the request sent to it.

So a request to this url: <http://localhost:8100/?data=B&imei=300434065264590&username=myuser> will return `FAILED,11, No RockBLOCK with this IMEI found on your account` since `data=B` and B is hex for 11 which is the corresponding error code as mentioned above.
