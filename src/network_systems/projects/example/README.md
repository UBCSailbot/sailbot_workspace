# Example - Cached Fibonacci

This is a simple example program to showcase and evaluate the workspace structure for Network Systems.

Run it with `ros2 run network_systems example --ros-args -p enabled:=true`.

Alternatively, using launch files run `ros2 launch network_systems main_launch.py config:=example/example_en.yaml`.

Publish to the topic using: `ros2 topic pub cached_fib_in std_msgs/msg/UInt64 "{data: <some number>}" --once`. Run it
without the `--once` flag to repeatedly publish.

The output will be shown in the terminal where the example is running. You can also see the output by running
`ros2 topic echo cached_fib_out` before sending the publish command.
