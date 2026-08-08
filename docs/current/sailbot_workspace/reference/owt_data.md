# Archiving On-Water Test Data

After an on-water test (OWT), the data it produced is processed and archived so
that it can be analyzed later and replayed by our software. The data comes from
two sources, and ends up in two destinations:

<!-- markdownlint-disable MD013 -->
| Source | Produces |
| --- | --- |
| [`polaris-gui`](https://github.com/UBCSailbot/polaris-gui){target=_blank} (ELEC laptop) | raw CAN dumps of the test |
| The boat computer (SOFT) | the combined ROS log and the rosbag recordings |

| Destination | Purpose |
| --- | --- |
| [`OWT-data`](https://github.com/UBCSailbot/OWT-data){target=_blank} | version-controlled archive the software reads from |
| Google Drive | full archive, including files too large for GitHub |
<!-- markdownlint-enable MD013 -->

## 1. Pull the CAN dumps from polaris-gui

The GUI writes one timestamped candump per run to the `logs/` folder of the
`polaris-gui` checkout on the laptop that ran it, rolling older ones into
`logs-arch/`:

```text
logs/candump_<YYYYMMDD>_<HHMMSS>.csv
```

Both folders are gitignored, so these files only exist on that laptop — copy
them off it before anything else. Each file is a slice of the same test, with
the columns `Timestamp,Elapsed_Time_s,CAN_Message`.

## 2. Extract the software logs from the boat computer

The combined log and the rosbags are written inside the deployment container.
Copy them out with `docker cp`, changing `<name>` to the container name:

<!-- markdownlint-disable MD013 -->
```bash
# Combined log: combined_log_<date>.txt
docker cp <name>:/workspaces/sailbot_workspace/src/global_launch/voyage_log ./voyage_log

# Rosbag recordings: .db3 database files
docker cp <name>:/workspaces/sailbot_workspace/notebooks/local_pathfinding/session_recording ./session_recording
```
<!-- markdownlint-enable MD013 -->

<!-- markdownlint-disable MD013 -->
See the [deployment runbook](https://github.com/UBCSailbot/sailbot_workspace/blob/main/.devcontainer/release/README.md){target=_blank}
for the surrounding deployment steps.
<!-- markdownlint-enable MD013 -->

## 3. Run polaris-data-analysis on the CAN dumps

<!-- markdownlint-disable MD013 -->
[`polaris-data-analysis`](https://github.com/UBCSailbot/polaris-data-analysis){target=_blank}
merges and decodes the dumps. Set it up once:
<!-- markdownlint-enable MD013 -->

```bash
python3 -m venv venv
source venv/bin/activate
python3 -m pip install -e . -r requirements.txt
```

Then drop the candumps from step 1 into a session folder under `data/`, named
for the test (for example `data/26Jun6_owt/`), and run:

```bash
# Merge the session's dumps into one chronological CAN log
python3 combine_session_logs.py --data-dir data/26Jun6_owt

# Decode that log into physical signal values
python3 decode_all_candumps.py --data-dir data/26Jun6_owt

# Render the dashboards
python3 build_physical_dashboard.py --data-dir data/26Jun6_owt
```

Each writes into `outputs/<session>/`:

<!-- markdownlint-disable MD013 -->
| File | Contents |
| --- | --- |
| `combined_can_frames.csv` | every CAN frame of the session, merged and ordered |
| `decoded_signals.csv` | the same frames decoded into physical values |
| `*_dashboard.png` | the physical, electrical, and sensor dashboards |
<!-- markdownlint-enable MD013 -->

!!! tip "Trim to the in-water window"

    The `combined_can_frames.csv` files committed to `OWT-data` are trimmed to
    the window where the boat was actually in the water, so that replaying one
    does not spend its first stretch on the dock.

## 4. Commit to OWT-data

Copy `OWT-yyyy-mm-dd/ [EXAMPLE]` in the
[`OWT-data`](https://github.com/UBCSailbot/OWT-data){target=_blank} repository
as the template for the new session, name it for the date of the test, and fill
it in:

```text
OWT-yyyy-mm-dd/
├── can_messages/
│   └── combined_can_frames.csv   # from step 3
├── voyage_log/
│   └── <timestamp>.txt           # the combined log from step 2
├── session_recordings/
│   └── datarecording_<timestamp>_0.db3
└── db_<Mon>_<D>.zip              # the zipped database files
```

!!! warning "decoded_signals.csv is not committed"

    Decoded logs run to hundreds of MB, which exceeds GitHub's 100 MB file
    limit, so they are gitignored in `OWT-data` and regenerated locally from
    `combined_can_frames.csv` when needed.

Note in the repository's README if the session's data was recorded with the
wrong timestamp, as has happened on past tests.

### Why this file is committed

`combined_can_frames.csv` is the one artifact the software reads back. Once a
session is committed, `sailbot_workspace` can pull it in and replay it through
the CAN transceiver, so Pathfinding and Controls can be exercised against real
sensor data from the test without a boat. This is why the committed copies are
trimmed to the in-water window, and why the file is worth committing even
though the decoded form is not.

!!! note "Pulling a can on-water session back into sailbot_workspace"

    The tooling for this lands with the mock CAN transceiver in network system

    That branch adds `scripts/get_mock_can_msg.sh`, which downloads a session's
    `combined_can_frames.csv` into `src/network_systems/lib/can_log/`:

    ```bash
    ./scripts/get_mock_can_msg.sh -d 2026-06-06
    ```

    It is also wired up as the **Pull Mock CAN Log** VS Code task, which
    prompts for the session date:
    `CTRL + SHIFT + P` > `Tasks: Run Task` > `Pull Mock CAN Log`.

    The date must be a session that exists in `OWT-data` and has a
    `can_messages/combined_can_frames.csv` — which is exactly what step 4
    produces. When you archive a new session, add its date to the task's
    `owt_date` options in `sailbot.code-workspace` so it shows up in the
    prompt.

## 5. Upload to Google Drive

Google Drive holds the full archive, including the files that are too large to
commit. In the `.OWT_Data` folder, create a folder named for the date of the
test and split it by sub-team:

```text
.OWT_Data/
└── DayMonthYear/
    ├── soft/
    │   ├── combined_log.txt          # the combined log from launch
    │   └── <database files>          # the rosbag recordings
    └── ELEC/
        ├── combined_can_frames.csv   # the merged dump from step 3
        ├── <raw candumps>            # the per-run dumps from step 1
        ├── physical_dashboard.png    # the dashboards rendered in step 3
        ├── electrical_dashboard.png
        └── sensor_dashboard.png
```

The dashboards are the generated images from `outputs/<session>/` in step 3.
Unlike `decoded_signals.csv`, they are small enough to keep anywhere, but Drive
is where the full set for a session lives.
