# Gazebo Dev Image

## Features

- Builds off the [`dev`](../base-dev/README.md) image and installs Gazebo
  Harmonic and a prebuilt `asv_wave_sim`, for the `boat_simulator_gazebo`
  physics backend
- Published separately from `dev` (as
  `ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev`) so that most developers,
  who never run the simulator, never pull or build its ~1.8 GB dependency
  closure. Opting in (`INSTALL_GAZEBO` in
  [`../docker-compose.yml`](../docker-compose.yml)) selects this image as the
  base in [`../Dockerfile`](../Dockerfile) instead of compiling
  `asv_wave_sim` (~15 minutes) locally on every rebuild

## How to build

1. In [`gazebo.Dockerfile`](gazebo.Dockerfile), make your changes and push
2. In the
   [Build gazebo-dev Image workflow](https://github.com/UBCSailbot/sailbot_workspace/actions/workflows/build-gazebo-dev-image.yml),
   select "Run workflow"
3. In "Use workflow from", select the branch that your changes are in
4. Update "Tag to publish the gazebo-dev image as" with a short description
   of the changes; for example, `add-gazebo-headless`
5. If your changes also depend on a newer `dev` image, update "Tag of
   ghcr.io/ubcsailbot/sailbot_workspace/dev to build on top of" to match;
   otherwise leave it at the default
6. Click "Run workflow"
7. In [`../Dockerfile`](../Dockerfile), update the `GAZEBO_TAG` build arg
   default with the short description above
8. Once the workflow successfully completes, run the "Dev Containers:
   Rebuild Container" VS Code command to use the newly built image (with
   `INSTALL_GAZEBO` uncommented in `../docker-compose.yml`)

### Debugging locally

Build a test image directly against `gazebo.Dockerfile`, the same way as
described in [`../base-dev/README.md`](../base-dev/README.md#debugging-locally),
substituting `gazebo.Dockerfile` for `base-dev.Dockerfile` and `gazebo-dev`
for `dev`.
