# Deploying a release to the Raspberry Pi

This guide covers building a release image of `sailbot_workspace` and deploying
it to the onboard Raspberry Pi. There are three build paths:

- **Path A — CI build + tar transfer** (recommended; Pi offline)
- **Path B — CI build + registry pull** (Pi has internet; take a long time to build)
- **Path C — local build + tar transfer** (fallback; requires native arm64 preferably)

All three produce the same `linux/arm64` release image. Paths A and B build it
in GitHub Actions on a native arm64 runner (~6 min), which is faster and more
reliable than building locally.

> **Build on arm64, not under emulation.** The release image is `linux/arm64`.
> Building it on an `amd64` machine uses QEMU emulation, which is ~5× slower
> (~30 min) and segfaults on the `ldconfig` step of the ROS dependency install.
> CI runs on `ubuntu-24.04-arm` (native), so prefer Paths A/B. Only build
> locally (Path C) on real arm64 hardware: an Apple Silicon Mac, the Pi itself,
> or an arm64 cloud VM.

---

## Paths A & B — CI build (shared first step)

Both paths start with a single CI workflow run that builds the image natively on
arm64, pushes it to the registry, and saves a tar artifact in parallel (~6 min).

There are two ways in running the CI. One is manually and the other is through a
software release on GitHub. It is preferable to create a release so that we can
track what version of the software we used for deployment and the changes
compared to the previous release.

**For running CI through a release (preferred):**

1. On GitHub: **Releases** → **Draft a new release** → **Choose a tag** → type a
   new tag (e.g. `on-water-8`) → **Publish release**.

2. The **Registers Built Image and Produces Tar File** workflow triggers
   automatically. Both jobs run in parallel:
   - **build-and-push** — pushes to `ghcr.io/ubcsailbot/sailbot_workspace/release:<tag>`
      and `:latest`
   - **build-tar** — uploads `release.tar` as a downloadable artifact (`release-<tag>-tar`)

3. Once both jobs pass, continue with **Path A** or **Path B** below.

**For manual CI build:**

1. Push your changes to your branch.

2. On GitHub: **Actions** tab → **Registers Built Image and Produces Tar File**
   workflow →
   **Run workflow** → change the branch from `main` to your branch, enter a tag
   (e.g. `on-water-8`) → **Run workflow**.

   The workflow runs two jobs in parallel:
   - **build-and-push** — pushes to
      `ghcr.io/ubcsailbot/sailbot_workspace/release:<tag>` and `:latest`
   - **build-tar** — uploads `release.tar` as a downloadable artifact (`release-<tag>-tar`)

3. Once both jobs pass, continue with **Path A** or **Path B** below.

---

## Path A — tar transfer (Pi offline)

1. **Download the artifact.** On the finished run's page, download the
   `release-<tag>-tar` artifact (it arrives zipped; unzip to get `release.tar`).
   Or from the CLI:

   ```bash
   gh run download <run-id> -n release-<tag>-tar
   ```

2. **Transfer to the Pi** (see [Reaching the Pi](#reaching-the-pi) for which
   address to use):

   ```bash
   # On the boat LAN/raye's wifi:
   rsync -a release.tar sailbot@192.168.0.10:/home/sailbot/

   # Remote (Tailscale):
   rsync -a release.tar sailbot@100.95.219.3:/home/sailbot/
   ```

3. **On the Pi**, load the image and reclaim space:

   ```bash
   docker load -i release.tar
   rm release.tar
   ```

4. **Run the container** (see [Run the container](#run-the-container)).

---

## Path B — registry pull (Pi has internet)

1. **Get a GitHub token with `read:packages`.** A fine-grained PAT scoped to the
   `ubcsailbot` packages (read-only, with an expiry) is preferred over a broad
   classic token. The package is private, so an unauthenticated pull returns
   `unauthorized`.

2. **On the Pi**, log in and pull:

   ```bash
   echo "$CR_PAT" | docker login ghcr.io -u <github-username> --password-stdin
   docker pull ghcr.io/ubcsailbot/sailbot_workspace/release:on-water-8

   # Optional: give it a tidier local name
   docker tag ghcr.io/ubcsailbot/sailbot_workspace/release:on-water-8 release
   ```

3. **Run the container** (see [Run the container](#run-the-container)). On a
   shared boat computer, run `docker logout ghcr.io` afterwards so the token
   doesn't persist in `~/.docker/config.json`.

---

## Path C — local build + tar transfer (fallback)

Only on native arm64 hardware (see the emulation note above). From the repo root
(`sailbot_workspace/`):

```bash
docker build \
  --platform linux/arm64 \
  -f .devcontainer/release/release.Dockerfile \
  --build-arg CACHEBUST=$(date +%s%3N) \
  -t release:on-water-8 \
  .

# Save the image as a .tar to transfer via rsync
docker save -o release.tar release:on-water-8
```

Then transfer and load exactly as in Path A, steps 2–3.

---

## Run the container

Run on the Pi. This starts the container and drops you into a bash shell inside
it (exit with `exit`). `--network host` and `--privileged` are required for CAN,
serial, and GPIO access.

```bash
docker run \
  --name <name> \
  -it --network host --privileged \
  release:on-water-8 \
  bash
```

### Redeploying (important)

A container is pinned to the image it was created from. Pulling/loading a new
image and then running `docker start <name>` restarts the **old** code, and
re-running `docker run --name <name>` fails with "name already in use". Before
redeploying, remove the old container first:

```bash
docker rm -f <name>       # remove the old container
docker image prune -f     # occasionally, so old images don't fill the SD card
# then docker run ... the new image
```

---

## Starting the software once deployed

Enter the container and start all ROS nodes in production, tee-ing a combined
log. Change `<name>` accordingly:

```bash
docker start <name> && \
docker exec -it <name> bash -ic "ros2 launch \
  src/global_launch/main_launch.py record:=true mode:=production \
  2>&1 | tee src/global_launch/voyage_log/combined_log_\$(date +%F_%H-%M-%S).txt"
```

> The launch arguments (`record`, `mode`, etc.) mirror behaviour that changes
> over time — treat the
> [global launch README](https://github.com/UBCSailbot/sailbot_workspace/blob/main/src/global_launch/README.md)
> as the source of truth and keep this runbook in sync.

To enter the container **without** starting the ROS nodes:

```bash
docker start -i <name>
```

### Extracting logs and bags

```bash
# Voyage logs
docker cp <name>:/workspaces/sailbot_workspace/src/global_launch/voyage_log ./voyage_log

# ROS 2 bags
docker cp <name>:/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recording ./session_recording
```
