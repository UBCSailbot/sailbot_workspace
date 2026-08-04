# WSL2 GPU Passthrough

## What it is

An opt-in compose overlay that lets OpenGL GUIs inside the Dev Container - the
Gazebo GUI, RViz - render on the Windows GPU instead of on the CPU.

By default the container has no GPU at all: no `/dev/dri`, no vendor userspace
driver. The hardware GLX path fails outright rather than degrading, so
[`../docker-compose.yml`](../docker-compose.yml) sets
`LIBGL_ALWAYS_SOFTWARE=1` to force Mesa's `llvmpipe` software rasterizer. That
keeps GUIs alive but makes them very slow. The Gazebo GUI is the worst case: the
`asv_wave_sim` ocean is a 256x256 FFT mesh regenerated at 30 Hz, which pins two
CPU cores and leaves the window drawing at a fraction of a frame per second -
usually mistaken for the GUI failing to load.

WSL2 already exposes the Windows GPU to Linux; the container is one mount away
from using it:

| Piece                                | Status without this overlay      |
| ------------------------------------ | -------------------------------- |
| `/dev/dxg`                           | present (via `privileged: true`) |
| Mesa d3d12 driver (`d3d12_dri.so`)   | present in the image             |
| `/usr/lib/wsl/lib` (`libd3d12core.so`, `libdxcore.so`, ...) | **missing**    |

Windows injects `/usr/lib/wsl` into every WSL2 distro, but Docker does not carry
it into containers. This overlay bind-mounts it read-only, points
`GALLIUM_DRIVER` at d3d12, and sets `LIBGL_ALWAYS_SOFTWARE` back to `0`.

## Requirements

- A **WSL2** host with a GPU that has WSL drivers (any reasonably current
  NVIDIA, AMD, or Intel Windows driver)
- `/usr/lib/wsl/lib` non-empty in the WSL distro Docker runs in. Check from a
  WSL shell on the host, outside the container:

  ```bash
  ls /usr/lib/wsl/lib
  ```

  It should list `libd3d12core.so`, `libdxcore.so`, and similar.

**This overlay is WSL2-only.** On macOS or a native Linux host `/usr/lib/wsl`
does not exist, and Docker creates an empty directory for a missing bind-mount
source instead of failing. The mount would silently succeed, `d3d12` would find
nothing, and `LIBGL_ALWAYS_SOFTWARE=0` would leave the GUI with no working
driver - a hard abort on startup instead of a slow window. Leave it disabled
there.

## How to enable it

Add the overlay to `dockerComposeFile` in
[`../devcontainer.json`](../devcontainer.json) by uncommenting this line:

```jsonc
"wsl/docker-compose.wsl.yml", // WSL2 GPU passthrough for OpenGL GUIs (WSL2 hosts only)
```

then run the "Dev Containers: Rebuild Container" VS Code command.

It must stay **after** `docker-compose.yml` in the list, since that is the file
whose `LIBGL_ALWAYS_SOFTWARE` it overrides. Compose applies files in order and
later ones win.

Like the Gazebo overlay, it ships commented out and CI asserts it stays that
way (see the `gazebo-disabled-by-default` job in
[`../../.github/workflows/tests.yml`](../../.github/workflows/tests.yml)):
enabling it for everyone would break every non-WSL host and every CI job.
Uncomment it locally, and revert before committing.

## Verifying it worked

After the rebuild, run any OpenGL GUI and check which renderer Ogre selected:

```bash
gz sim -r shapes.sdf              # then, in another terminal:
grep -E "GL_RENDERER|GL_VENDOR" ~/.gz/rendering/ogre2.log
```

Working:

```text
GL_RENDERER = D3D12 (<your GPU name>)
```

Still on the CPU:

```text
GL_RENDERER = llvmpipe (LLVM 15.0.7, 256 bits)
```

If it is still `llvmpipe`, check `ls /usr/lib/wsl/lib` **inside** the container -
an empty directory means the bind-mount source did not exist on the host.

## Notes

- `LD_LIBRARY_PATH` is set to `/usr/lib/wsl/lib:/opt/ros/humble/lib`. The ROS
  path is not redundant: a compose `environment:` value replaces the image's
  `ENV` rather than extending it, and
  [`../base-dev/base-dev.Dockerfile`](../base-dev/base-dev.Dockerfile) sets
  `LD_LIBRARY_PATH=/opt/ros/humble/lib`. Dropping it breaks library resolution
  for ROS binaries.
- GPU access does not reduce the WSL VM's memory limit, which the Gazebo GUI can
  otherwise exhaust (it holds ~1 GB resident on top of the server). If the VM
  has less than about 8 GB, raise it in `%UserProfile%\.wslconfig` on Windows.
- This overlay is independent of the Gazebo overlay. Enabling both is the normal
  setup for running the simulator with its GUI on Windows; enabling only this one
  still speeds up RViz and any other OpenGL tool.
