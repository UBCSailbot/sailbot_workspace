# Gazebo Dev Image

## Features

- Builds off the [`dev`](../base-dev/README.md) image and installs Gazebo
  Harmonic and a prebuilt `asv_wave_sim`, for the `boat_simulator_gazebo`
  physics backend
- Published separately from `dev` (as
  `ghcr.io/ubcsailbot/sailbot_workspace/gazebo-dev`) so that most developers,
  who never run the simulator, never pull or build its ~1.8 GB dependency
  closure. Opting in (see [How to enable it](#how-to-enable-it)) selects this
  image as the base in [`../Dockerfile`](../Dockerfile) instead of compiling
  `asv_wave_sim` (~15 minutes) locally on every rebuild
- Published for `linux/amd64` only. OSRF does not publish the
  `ros-humble-ros-gzharmonic*` bridge packages this image needs for `arm64`
  on jammy (only the bare `gz-harmonic` engine), so it cannot be built
  natively for Apple Silicon - Docker Desktop falls back to emulating the
  `amd64` image there instead

## How to enable it

Add the overlay to `dockerComposeFile` in
[`../devcontainer.json`](../devcontainer.json) by uncommenting this line:

```jsonc
"gazebo/docker-compose.gazebo.yml", // Gazebo simulator backend (amd64 only)
```

then run the "Dev Containers: Rebuild Container" VS Code command.

[`docker-compose.gazebo.yml`](docker-compose.gazebo.yml) sets the
`INSTALL_GAZEBO` build arg to `"true"`, which switches the base image in
[`../Dockerfile`](../Dockerfile) over to `gazebo-dev`, and pins the service to
`linux/amd64`. The platform pin is what makes this work on Apple Silicon:
without it Docker targets the host's native `arm64`, finds no `arm64` variant
of `gazebo-dev`, and fails with "no match for platform in manifest" instead of
falling back to emulation. Expect it to be slow there - the sim runs
translated, on top of software rendering that already has no GPU inside Docker
on macOS.

Editing `INSTALL_GAZEBO` directly in [`../docker-compose.yml`](../docker-compose.yml)
also works, but the overlay is preferred: it leaves the base file at `"false"`,
which is what the `gazebo-disabled-by-default` CI check enforces, so there is
no local edit to remember to revert before committing.

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
   Rebuild Container" VS Code command to use the newly built image (with the
   overlay enabled, per [How to enable it](#how-to-enable-it))

### Debugging locally

Build a test image directly against `gazebo.Dockerfile`, the same way as
described in [`../base-dev/README.md`](../base-dev/README.md#debugging-locally),
substituting `gazebo.Dockerfile` for `base-dev.Dockerfile` and `gazebo-dev`
for `dev`.
