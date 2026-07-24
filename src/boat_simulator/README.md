# UBC Sailbot Boat Simulator

UBC Sailbot's boat simulator for the new project. This repository contains a
ROS package `boat_simulator`. This README contains only setup and run
instructions. Further information on the boat simulator can be found on the
<!-- markdownlint-disable-next-line MD013 -->
software team's [docs website](https://ubcsailbot.github.io/sailbot_workspace/main/current/boat_simulator/overview/).

## Setup

<!-- markdownlint-disable-next-line MD013 -->
The boat simulator is meant to be ran inside the [Sailbot Workspace](https://github.com/UBCSailbot/sailbot_workspace)
development environment. Follow the setup instructions for the Sailbot
Workspace
<!-- markdownlint-disable-next-line MD013 -->
[here](https://ubcsailbot.github.io/sailbot_workspace/main/current/sailbot_workspace/usage/setup/)
to get started and build all the necessary ROS packages.

## Run

<!-- markdownlint-disable-next-line MD013 -->
The [`launch/`](./launch/) folder contains a [ROS 2 launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
responsible for starting up the boat simulator. To run the boat simulator
standalone, execute the launch file after building the `boat_simulator`
package:

``` shell
ros2 launch boat_simulator main_launch.py [OPTIONS]...
```

To see a list of options for simulator configuration, add the `-s` flag at the
end of the above command.

## Regenerating airfoil coefficient data

The simulator's lift and drag forces depend on each foil's coefficients, which
vary with both the angle of attack, the Reynolds number (`Re = |v| * chord /
kinematic_viscosity`) and the mach number. For our calculations, we will consider the Reynolds number and angle of
attack Rather than a single hand-tuned curve per foil, the coefficients live in
[`common/airfoil_polars.py`](./boat_simulator/common/airfoil_polars.py) as a set of real airfoil polars
— one per Reynolds number — that the force computation selects from at runtime by picking the closest
tabulated Reynolds number.

That module is **generated**, not edited by hand. The
[`scripts/build_airfoil_polars.py`](./scripts/build_airfoil_polars.py) script downloads XFOIL polars from
[airfoiltools.com](http://airfoiltools.com) for each foil's NACA section
(keel and rudder use NACA 0012; wingsail and trim tab use NACA 0018) at Reynolds numbers 50k–1M,
resamples the pre-stall branch to whole-degree angles of attack, and extends each polar out to 90° with the
Viterna–Corrigan post-stall model
([More details on Viterna extrapolation](https://www.simis.io/docs/aerodynamic-loads-viterna-extrapolation))
(a sail sits near 90° when running dead downwind, well beyond XFOIL's ~±18° range).

Regenerate the data module after changing which foils, sections, Reynolds
numbers, or aspect ratios are modeled (all configured near the top of the
script). It requires network access and is never imported at runtime:

``` shell
python3 src/boat_simulator/scripts/build_airfoil_polars.py
```

## Test

Run the `test` task in the Sailbot Workspace. See
<!-- markdownlint-disable-next-line MD013 -->
[here](https://code.visualstudio.com/docs/getstarted/userinterface#_command-palette)
on how to run vscode tasks.
