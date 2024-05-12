# Scripts

## `ament-lint.sh`

Script to lint source code in all ROS packages.

## `build.sh`

Script to build all ROS packages in the Sailbot Workspace.

## `clang-tidy.sh`

Script to run Clang-Tidy using `ament_clang_tidy.py`.

## `run_virtual_iridium.sh`

```shell
./run_virtual_iridium.sh <(optional) webhook server url> <(optional) virtual iridium http server port>
```

Creates a pair of socat sockets `$LOCAL_TRANSCEIVER_TEST_PORT` and `$VIRTUAL_IRIDIUM_PORT` and binds the latter to a
virtual iridium server running on localhost:8080, which substitutes the Rockblock HTTP server used in deployment.
Allows testing of satellite code without needing physical hardware.

Optional argument - webhook server url:

- Specify where the URL where the Remote Transceiver or whatever other HTTP server is running.
- Default is `http://127.0.0.1:8081`, which assumes fully local testing.

Optional argument - virtual iridium server port

- Specify which localhost port the virtual iridium runs on.
- Default is 8080.

`$LOCAL_TRANSCEIVER_TEST_PORT` acts as the serial port for AT commands. For example, to test via CLI:

1. `./run_virtual_iridium.sh`
2. To monitor just the `$LOCAL_TRANSCEIVER_TEST_PORT` without extra debug messages, in a new terminal run
    `cat $LOCAL_TRANSCEIVER_TEST_PORT`. What you see output from this command will be what the Local Transceiver reads
    and sends.
3. To issue CLI commands, open a new terminal and run `stty 19200 < $LOCAL_TRANSCEIVER_TEST_PORT` to set the baud rate.
4. `printf "at+sbdix\r" > $LOCAL_TRANSCEIVER_TEST_PORT`. This command queries the (currently empty) mailbox.
5. `curl -X POST -F "test=1234" http://localhost:8080` (this is garbage data - it doesn't mean
    anything). You should see the original terminal print that it received a POST request.
6. `printf "at+sbdix\r" > $LOCAL_TRANSCEIVER_TEST_PORT` to view the mailbox again. It will now indicate that it has the
    data.

Other relevant commands include (but are not limited to):

- `at+sbdwb=<msg_length>\r`: Setup the port to receive binary data of length `msg_length` on next input.
- `at+sbdrb\r`: Read binary content in the mailbox.
- `at+sbdd2\r`: Clear all buffers.


## `run-tests.sh`

Script to setup, build, and test all ROS packages.

## `setup.sh`

Script to handle ROS setup.

## `test.sh`

Script to run tests in all ROS packages.
