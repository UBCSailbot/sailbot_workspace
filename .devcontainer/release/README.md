# Deploying a release to the Raspberry Pi

This guide covers building a release image of `sailbot_workspace` and deploying
it to the onboard Raspberry Pi. There are four build paths:

- **Path A — CI build + tar transfer** (recommended; Pi offline)
- **Path B — CI build + registry pull** (Pi has internet; takes a long time to build)
- **Path C — local build + tar transfer** (fallback; requires native arm64 preferably)
- **Path D —  local build on `.devcontainer` with through Git Pull (fallback during testing; Pi offline)

Paths A, B, and C produce the same `linux/arm64` release image. Paths A and B
build it in GitHub Actions on a native arm64 runner (~6 min), which is faster and
more reliable than building locally.

Path D interacts with the existing `.devcontainer` on the Pi and involves
running `scripts/build.sh` (~13 minutes).

If you only need to move **code changes** and not a whole new image, skip the
build paths entirely — see
[Updating source without internet](#updating-source-without-internet-pi-offline).

> **Build on arm64, not under emulation.** The release image is `linux/arm64`.
> Building it on an `amd64` machine uses QEMU emulation, which is ~5× slower
> (~30 min) and segfaults on the `ldconfig` step of the ROS dependency install.
> CI runs on `ubuntu-24.04-arm` (native), so prefer Paths A/B. Only build
> locally (Path C) on real arm64 hardware: an Apple Silicon Mac, the Pi itself,
> or an arm64 cloud VM.

---

## Paths A & B — CI build (shared first step)

Both paths start with a single CI workflow run that builds the image natively on
arm64, pushes it to the registry, and saves a tar artifact in parallel
(for Path A). This takes about 6 minutes.

There are two ways of running the CI. One is manually and the other is through a
software release on GitHub. It is preferable to create a release so that we can
track what version of the software we used for deployment and the changes
compared to the previous release.

**For running CI through a release (preferred):**

1. On GitHub: **Releases** → **Draft a new release** → **Choose a tag** → type a
   new tag (e.g. `on-water-8`) → **Publish release**.

2. The workflow **Registers Built Image and Produces Tar File** triggers
   automatically. Both jobs run in parallel:
   - **build-and-push** — pushes to `ghcr.io/ubcsailbot/sailbot_workspace/release:<tag>`
      and `:latest`
   - **build-tar** — uploads `release.tar` as a downloadable artifact (`release-<tag>-tar`)

3. Once both jobs pass, continue with
[**Path A**](#path-a--tar-transfer-pi-offline) or
[**Path B**](#path-b--registry-pull-pi-has-internet) below.

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

3. Once both jobs pass, continue with
[**Path A**](#path-a--tar-transfer-pi-offline) or
[**Path B**](#path-b--registry-pull-pi-has-internet) below.

---

## Path A — tar transfer (Pi offline)

<!-- markdownlint-disable MD013 -->

1. **Download the artifact.** On the finished run's page, download the
   `release-<tag>-tar` artifact (it arrives zipped; unzip to get `release.tar`).
   Or from the CLI:

   ```bash
   gh run download <run-id> -n release-<tag>-tar
   ```

2. **Transfer to the Pi** (see [Reaching the Pi](https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1791230669/Raspberry+Pi+Main+Computer+Setup) for which address to use):

   ```bash
   # On the boat LAN/raye's wifi:
   rsync -a release.tar sailbot@192.168.0.10:/home/sailbot/

   # Remote (Tailscale):
   rsync -a release.tar sailbot@100.95.219.39:/home/sailbot/
   ```

   Use `rsync -aP` on a flaky PAN link — it shows progress and resumes a
   partial transfer instead of starting the multi-GB tar over.

3. **On the Pi**, load the image and reclaim space:

   ```bash
   docker load -i release.tar
   rm release.tar
   ```

4. **Run the container** (see [Run the container](#run-the-container)).

<!-- markdownlint-enable MD013 -->

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

If you'd rather not stage a tar on disk at either end, pipe it straight over
SSH — one command, no intermediate file:

```bash
docker save release:on-water-8 | gzip | \
  ssh sailbot@192.168.0.10 'gunzip | docker load'
```

This has no resume, so prefer the `rsync -aP` route on an unreliable link.

---

## Path D — local build on `.devcontainer` with through Git Pull (fallback)

Requires an SSH connection to the Pi. The dev container is named `owt-dev`.

Once connected, run the following:

```bash
docker start -i owt-dev
```

Once in the container run the following:

```bash
git pull # only works if the Pi has internet — otherwise see the section below

./scripts/run_software.sh
```

---

## Updating source without internet (Pi offline) through Git Pull

The Pi reaches the boat's private WiFi for SSH, but that network has no route to
GitHub, so `git pull` on the Pi fails. Git doesn't need the internet, though —
only a route to a machine holding the objects, and your laptop is that machine.
SSH over the PAN is a perfectly good transport.

Use this when only **code** changed. If the image itself changed (new apt/ROS
dependencies, Dockerfile edits), you need a full rebuild via Paths A–C. Code
baked into the release image can't be patched this way; this updates the repo
checked out on the Pi host at `~/sailbot_workspace`, which is what Path D's dev
container uses.

### Option 1 — push from laptop to Pi (usually easiest)

A normal repo refuses pushes to its checked-out branch. Set this once **on the
Pi**:

```bash
git -C ~/sailbot_workspace config receive.denyCurrentBranch updateInstead
```

Then from your laptop:

```bash
git remote add pi ssh://sailbot@192.168.0.10/home/sailbot/sailbot_workspace
git push pi HEAD:main
```

`updateInstead` also updates the Pi's working tree, but only if it's clean —
commit or stash anything on the Pi first. If you'd rather not set that config,
push to a scratch branch instead (`git push pi main:incoming`) and merge it on
the Pi.

### Option 2 — `git bundle` (also survives a USB stick)

A bundle is the same bytes `git push` would have sent, frozen into one file.
Git treats it as a read-only remote.

On your laptop:

```bash
# Full bundle — works even into an empty repo
git bundle create polaris.bundle --branches --tags

# Or incremental: only what the Pi doesn't already have
BASE=$(ssh sailbot@192.168.0.10 'git -C ~/sailbot_workspace rev-parse HEAD')
git bundle create polaris.bundle main ^$BASE

rsync -aP polaris.bundle sailbot@192.168.0.10:/home/sailbot/sailbot_workspace/
```

On the Pi:

```bash
cd ~/sailbot_workspace
git bundle verify ./polaris.bundle
git status                        # confirm clean
git pull ./polaris.bundle main
```

To inspect before merging rather than pulling straight in:

```bash
git fetch ./polaris.bundle main:refs/remotes/bundle/main
git log --oneline main..bundle/main
git merge bundle/main             # or: git reset --hard bundle/main
```

An incremental bundle is typically kilobytes rather than tens of megabytes, so
prefer it for repeat trips. It's a snapshot — commit again and you regenerate.

### Gotchas

<!-- markdownlint-disable MD013 -->

| Symptom | Cause / fix |
| --- | --- |
| `does not appear to be a git repository` | Git couldn't read a bundle at that path, so it fell back to treating it as a repo directory. Check `ls -l` and that `head -c 16` shows `# v2 git bundle`. Always pass an explicit path (`./polaris.bundle`), never a bare filename. |
| `refusing to fetch into branch 'refs/heads/main' checked out at ...` | You can't fetch directly onto the checked-out branch. Use `git pull`, or fetch into `refs/remotes/bundle/main` and merge. |
| Later "dubious ownership" or permission errors | Someone ran `sudo git`, leaving root-owned objects in `.git`. Never `sudo git` here. Repair with `sudo chown -R sailbot:sailbot ~/sailbot_workspace`. |
| Bundle contains the wrong commits | Your laptop's local `main` had drifted from `origin/main`. Check `git bundle verify` output — if they differ, bundle `refs/remotes/origin/main` instead. Always fetch from GitHub *before* leaving the internet. |
| Interrupted transfer | Use `rsync -aP` (resumable) rather than `scp`. |

<!-- markdownlint-enable MD013 -->

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
  2>&1 | tee src/global_launch/voyage_log/combined_log_$(date +%F_%T).txt"
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

<!-- markdownlint-disable MD013 -->
```bash
# Voyage logs
docker cp <name>:/workspaces/sailbot_workspace/src/global_launch/voyage_log ./voyage_log

# ROS 2 bags
docker cp <name>:/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recording ./session_recording
```
<!-- markdownlint-enable MD013 -->

These files are one half of an on-water test's data; the other half is the CAN
dumps captured by the GUI. For what to do with them next — merging and decoding
the CAN dumps, then archiving everything in `OWT-data` and Google Drive — see
[Archiving On-Water Test Data](../../docs/current/sailbot_workspace/reference/owt_data.md).
