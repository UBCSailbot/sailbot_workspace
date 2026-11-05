# Remote Server Standalone

This folder contains a standalone version of the remote_transceiver server and sailbot_db,
decoupled from ROS. It is ready to be copied into a new repository and built independently.

## Structure

- `src/` - Source files
- `inc/` - Header files
- `lib/` - Third-party or static libraries (if needed)
- `scripts/` - Helper scripts

## Build

Use the provided CMakeLists.txt to build the project. All ROS dependencies
have been removed.

Note that a clean build (removing all dependencies etc.) takes a long time
and should be avoided. There's a chance the script won't fetch all packages
required and manual installations and setup may be required, especially for
the MongoDB CXX driver.

Just rebuilding any `.cpp` or `.h` file modifications by running `build.sh`
should suffice for most cases. This also takes a few minutes, but you should
see progress % updates every minute or so.

`build.sh` builds three programs into `build/`: `remote_server`, `sailbot_cli`
and `remote_server_tests`.

## Server

```bash
./scripts/run_server.sh             # stores data in sailbot_db (production)
./scripts/run_server.sh --db test   # stores data in the test database
```

Each start creates a new active voyage in the chosen database. Use
`--db test` when testing locally: a start against `sailbot_db` also switches
the voyage the deployed server stores real data under.

## Voyages

Data from different trips is kept apart by voyage, all inside the `sailbot_db`
database.

- The `voyages` collection holds one document per voyage:
  `{name, created_at, created_by}`. `created_by` is `server` or `cli`.
- The active voyage is stored in a single document,
  `active_voyage: {_id: "active", voyage}`. Switching it is one atomic write,
  so an upload never sees zero or two active voyages. Older voyage documents
  may still have an `active` field, which is ignored.
- Every document the server writes to the sensor collections (`gps`,
  `ais_ships`, `temp_sensors`, `ph_sensors`, `salinity_sensors`, `batteries`,
  `wind_sensors`, `local_path`) gets a `voyage` field with the name of the
  active voyage. If no voyage is active the field is left out, and the CLI
  shows these documents as `untagged`. `global_path` and `iridium_response`
  are not tagged.
- Sensor timestamps are stored percent-encoded, the way RockBLOCK sends them
  (`26-08-21%2022%3A46%3A04`). The server rewrites plain or form-encoded
  times (`26-08-21 22:46:04`) to that form, so `list` and `range` order and
  match every message the same way.
- Each time `remote_server` starts, it creates a new voyage named
  `YYYY-MM-DD_N` (UTC date, `N` counts up within a day) and makes it active.
  If the server restarts in the middle of a trip, use `activate` to switch back
  to the earlier voyage, or `rename` to merge the two.

## CLI

```bash
./scripts/run_cli.sh
```

The CLI asks for the MongoDB password and connects to `sailbot_db` on the
Atlas cluster. Type `help` to list the commands available where you are.

| Command | What it does |
| --- | --- |
| `collections` | Every collection, with total, tagged, untagged and bad-timestamp counts |
| `voyages` | Every voyage, with its message count and first and last message time |
| `newvoyage [name]` | Create a voyage and make it active (default name `YYYY-MM-DD_N`) |
| `activate <voyage>` | Send new incoming data to an existing voyage |
| `rename <old> <new>` | Rename a voyage. If `<new>` already exists, the two voyages are merged |
| `enter <voyage>` | Work inside one voyage (or `untagged`) until `quit` |
| `list [n\|all]` | The latest `n` messages (default 20), oldest first, numbered |
| `range <start> <end>` | Messages between two times, e.g. `range 2026-08-21 2026-08-22_12:30` |
| `show <#>` | Every field of one listed message, across all sensor collections |
| `assign <voyage> <rows>` | Move listed messages into a voyage (it can be created on the spot) |
| `unassign <rows>` | Remove the voyage from listed messages |
| `del <rows>` | Delete listed messages from every sensor collection |
| `delvoyage <voyage>` | Delete a voyage and all of its messages |
| `export <voyage>` | Write `<voyage>.csv` to the current folder |
| `quit` / `exit` | Leave the voyage / end the program |

Inside a voyage, `delvoyage` and `export` take no argument and act on that
voyage. A message is one Iridium upload: the documents in each sensor
collection that share its `timestamp` and `voyage`, so an upload replayed into
another voyage is a separate message. `list` and `range` number the messages,
and `<rows>` refers to those numbers: `3`, `1-5` or `1,4,7-9`. The `parts`
column shows how many of the 8 sensor collections have a document for that
message.

Safety rules:

- Every change is previewed and has to be confirmed. `del` asks you to type
  `delete`, `delvoyage` asks for the voyage name, and a merge asks for `merge`.
- The active voyage cannot be deleted. Activate another voyage first.
- Before anything is deleted or merged, the documents are written to
  `backup_<UTC time>/<collection>.jsonl` in the current folder. To restore one
  collection:

  ```bash
  mongoimport --uri "<connection string>" --db sailbot_db --collection gps --file backup_<UTC time>/gps.jsonl
  ```

  To undo a merge, restore every file of its backup with `--mode upsert`
  added, so the merged messages get their old voyage back. If the old voyage
  was active, `activate` it again.

The CSV has one row per GPS message. The first battery, wind, temperature, pH
and salinity reading with the same timestamp is added to the row:
`time,timestamp_raw,voyage,latitude,longitude,speed,heading,batt_voltage,batt_current,wind_speed,wind_direction,temp,ph,salinity`.
