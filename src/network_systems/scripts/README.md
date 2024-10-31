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

This web server is a virtual implementation of the Rockblock Web Servers used to send MT messages. The specification for
 the RockBlock Web Servers can be found
 [here](https://docs.groundcontrol.com/iot/rockblock/web-services/sending-mt-message).

This virtual server follows this specification with one notable difference, it will return the specific error code that
is contained in the `ec` section of the request sent to it.

So a request to this url: <http://localhost:8100/?data=thisistestdata&ec=B&imei=300434065264590&username=myuser> will return
`FAILED,11, No RockBLOCK with this IMEI found on your account` since `ec=B` and B is hex for 11 which is the
 corresponding error code as mentioned above.

Continuing on, the data parameter now implements the functionality for any form of data to be handled by the server. This data will be written to `TEMP_FILE_PATH` so that any tests can properly verify that the rockblock web server will have received the correct data. So as above, if all goes well, `thisistestdata` should be written to the file located at `TEMP_FILE_PATH`.
