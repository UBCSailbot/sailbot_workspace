<!-- markdownlint-disable MD013 -->

# POLARIS website architecture

A high-level overview of the POLARIS website (`src/website`): what it does end to end, its core pieces, and how data moves through it. Implementation details are deliberately left out. Diagrams live in [`diagrams/`](diagrams/).

## Purpose

The website was built to **livestream the POLARIS voyage to the public**: UBC Sailbot's autonomous sailboat, released off the west coast of Vancouver Island in August 2026.

It has two pages:

- **Map (`/`):** shows where the boat is right now and the path it has sailed so far, so a visitor can follow the voyage as it happens.
- **Downloads (`/download`):** lets visitors download the data collected during the voyage (GPS, nearby AIS ships, wind, temperature, pH, salinity, and the planned paths) as CSV, Excel, or JSON.

**Audience:** anyone. The site is fully public, with no accounts, logins, or admin features.

**Scope:** it was designed for **one boat on one voyage**. There's no concept of multiple voyages or boats.

## Tech stack

| Layer         | Technology                                               | Role                                                                                                                                                                                                                                  |
| ------------- | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Framework     | **Next.js** (TypeScript)                                 | React framework. Serves the pages and the read-only API in one app                                                                                                                                                                    |
| UI            | **React**                                                | Component library for building the dynamic frontend                                                                                                                                                                                   |
| Client state  | **Redux** + **Redux-Saga**                               | Redux stores and shares state across components. Sagas run the background loops that poll the API for new data                                                                                                                        |
| Map / charts  | **Leaflet** (CARTO map tiles), **uPlot**                 | Interactive voyage map; sensor charts (currently hidden)                                                                                                                                                                              |
| Database      | **MongoDB** + **Mongoose**                               | Stores boat data as documents. The data isn't relational enough to need SQL, and its shape changes as the boat and sensors change; MongoDB needs no migrations for that. Mongoose gives the website typed models over the collections |
| External data | **aisstream.io**                                         | Live feed of nearby ships (AIS), ingested by a separate worker script                                                                                                                                                                 |
| Local dev     | **Docker**                                               | Runs a local MongoDB for development                                                                                                                                                                                                  |
| Deployment    | **GitHub Actions**, **PM2** on a **DigitalOcean** server | Builds on every push to `main` and runs the production server                                                                                                                                                                         |

## Data

Diagrams: [data relationships (ER)](diagrams/data-er.md) · [data flow](diagrams/data-flow.md)

### Sources

The website reads all of its data from one MongoDB database. Three things write to it:

| Source                           | What it provides                                                                                                                     | How it arrives                                                                                                                                                  | How often                                                          |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| **The boat (POLARIS)**           | GPS, wind, batteries, temperature, pH, salinity, nearby ships seen by the boat's own AIS receiver, and the boat's current local path | Satellite (Iridium, via a RockBLOCK modem) → RockBLOCK's cloud forwards each message as an HTTP POST → **remote server** (C++) decodes it and writes to MongoDB | **Once per hour** by default (boat config `send_period_sec: 3600`) |
| **aisstream.io**                 | Live positions of ships around the boat (AIS)                                                                                        | WebSocket stream → **AIS ingest worker** (Node script) keeps the latest position per ship and writes a snapshot                                                 | Every 15 s, keeping only the newest 50 snapshots                   |
| **Simulation script** (dev only) | Fake versions of all of the above                                                                                                    | Python script writes JSON fixtures directly                                                                                                                     | On demand                                                          |

The website itself never writes to the database. It reads through its API and shows the results on the map and the downloads page.

### What the website uses

| Dataset                         | Contents of one reading                                                                     | Source                             | Used on                                 |
| ------------------------------- | ------------------------------------------------------------------------------------------- | ---------------------------------- | --------------------------------------- |
| **GPS**                         | Boat latitude, longitude, speed, heading                                                    | Boat                               | Map (boat position and path); downloads |
| **AIS ships**                   | List of nearby ships: ID (MMSI), position, speed and course over ground, rate of turn, size | Boat AIS **and** aisstream.io      | Map (ship markers); downloads           |
| **Local path**                  | List of waypoints the boat is currently steering toward                                     | Boat                               | Map (hidden); downloads                 |
| **Global path**                 | List of waypoints for the full planned route                                                | Simulation only (see known issues) | Map (hidden); downloads                 |
| **Wind sensors**                | List of sensors, each with speed and direction                                              | Boat                               | Charts (hidden); downloads              |
| **Temperature / pH / salinity** | List of readings, one per sensor                                                            | Boat                               | Charts (hidden); downloads              |
| **Batteries**                   | Two batteries, each with voltage and current                                                | Boat                               | Nothing (removed from the UI)           |

> **Current state:** the voyage is over, so the map no longer reads live GPS. It shows a hardcoded copy of the voyage path and a fixed last-known position.

### Form of the data at each stage

Boat data takes four forms on its way to a visitor: satellite message → database document → API JSON → downloaded file. The values below are illustrative.

**1. As received from the satellite.** RockBLOCK POSTs a form to the remote server. The sensor data is a binary protobuf message, hex-encoded:

```text
POST /sensors
imei=300434000000000&momsn=812&transmit_time=26-08-20 14:03:22
&iridium_latitude=48.8345&iridium_longitude=-125.1370&iridium_cep=3
&data=0d8b0b43421599b4fac21d...        (hex-encoded protobuf, truncated)
```

Decoded, one message holds one reading of everything (shown as JSON):

```json
{
  "gps": {
    "latitude": 48.8345,
    "longitude": -125.137,
    "speed": 2.1,
    "heading": 270
  },
  "wind_sensors": [
    { "speed": 15, "direction": -57 },
    { "speed": 11, "direction": -34 }
  ],
  "batteries": [
    { "voltage": 12.4, "current": 1.1 },
    { "voltage": 12.3, "current": 0.9 }
  ],
  "ais_ships": [
    {
      "id": 316001234,
      "latitude": 48.9,
      "longitude": -125.2,
      "sog": 12.5,
      "cog": 120,
      "rot": 0,
      "width": 30,
      "length": 80
    }
  ],
  "temp_sensors": [11.2],
  "ph_sensors": [8.1],
  "salinity_sensors": [31.5],
  "local_path_data": {
    "waypoints": [{ "latitude": 48.83, "longitude": -125.15 }]
  }
}
```

**2. As stored in MongoDB.** The remote server splits each message into **one document per collection**, and stamps each with the message's `transmit_time`:

```js
// gps
{ _id: ObjectId("..."), latitude: 48.8345, longitude: -125.137, speed: 2.1, heading: 270, timestamp: "26-08-20 14:03:22" }

// wind_sensors
{ _id: ObjectId("..."), windSensors: [{ speed: 15, direction: -57 }, { speed: 11, direction: -34 }], timestamp: "26-08-20 14:03:22" }

// aisships (written by the aisstream worker: a snapshot of every ship currently nearby)
{ _id: ObjectId("..."), ships: [{ id: 316001234, latitude: 48.9, longitude: -125.2, cog: 120, rot: 0, sog: 12.5, width: 30, length: 80 }, ...],
  timestamp: "2026-08-20T14:03:30.123Z" }
```

**3. As served by the website's API.** Every endpoint returns the whole collection (or the newest N documents) as JSON, without `_id`:

```text
GET /api/gps
{ "success": true, "data": [{ "latitude": 48.8345, "longitude": -125.137, "speed": 2.1, "heading": 270, "timestamp": "26-08-20 14:03:22" }, ...] }
```

**4. As downloaded.** CSV and Excel flatten the nested lists into one row per inner item, so a wind reading with two sensors becomes two rows (the timestamp appears only on the first):

```csv
timestamp,speed,direction
26-08-20 14:03:22,15,-57
,11,-34
```

### Characteristics that matter for the database

- **Append-only time series.** Every write is a new reading. Nothing is updated or deleted, except the AIS worker trimming old snapshots.
- **One document per reading, with nested lists.** A reading naturally contains a variable-length list (sensors, ships, waypoints). That fits a document better than a table plus a child table.
- **The shape changes with the hardware.** Over time, generic sensors were replaced by temperature/pH/salinity, and pressure was dropped. In MongoDB, new documents simply take the new shape, without migrations.
- **Readings are linked only by timestamp.** All documents from one satellite message share the same `timestamp` string. There's no message ID, voyage ID, or boat ID, so "everything from voyage X" or "everything from boat Y" can't be queried.
- **No shared schema between writers.** The C++ server, the AIS worker, and the website's models each define the data shape independently, and MongoDB enforces none of them (see known issues).

### Known data issues

- **Global path never arrives from the boat.** The remote server has code to store a global path, but nothing calls it. The website also reads `globalpaths` while the server would write `global_path`.
- **Two AIS sources, two collections.** The boat's AIS goes to `ais_ships`. The aisstream worker writes to `aisships`. The website reads only `ais_ships`.
- **Two timestamp formats.** The boat path uses `YY-MM-DD HH:MM:SS` (RockBLOCK's transmit time, UTC). The AIS worker and Mongoose default use ISO 8601. Both are stored as strings, not dates.

## Core functionality and Constraints

The end-to-end interactions the website exists for. Each has a sequence diagram in [`diagrams/`](diagrams/). (More core functionality may be added; this list covers what has been mapped so far.)

### CF1: View the voyage on the map

**Supports the purpose by:** (visually) being a livestream of the boats location and other vessels around it.

Currently data like the voyage path and the cursor are actually hardcoded and are not livestreamed. Even if they weren't hardcoded, they are updated hourly by the boat so livestreaming wouldn't serve much purpose.

If the boat consistently updated this data, then we could change to a livestreaming approach or poll for data more frequently.

See [`diagrams/cf1-map-sequence.md`](diagrams/cf1-map-sequence.md) for the end-to-end sequence diagram.

### CF2: Download a dataset

**Supports the purpose by:** making the voyage's data available to anyone who wants it in tabular format, not just viewable on the map.

This data is pulled from the database which the boat writes to hourly. There isn't much point in livestreaming this data because the updates happen hourly.

If the boat consistently updated this data, then we could change to a livestreaming approach or poll for data more frequently.

See [`diagrams/cf2-download-sequence.md`](diagrams/cf2-download-sequence.md) for the end-to-end sequence diagram.

## Improvements to Explore

### Livestreaming:

If Zephyr wants to add livestreaming to the website, then we'll have the frequency the boat sends messages to the C++ database. Currently, boat messages are only sent once per hour, so the best the website can do for dataset and position updates is hourly.

If the boat sends messages more frequently, then we need to change some of our architecture. We'll need to change our one hour polling frequency to match the upload rate of the boat. Parsing these messages into collection documents and how we query them should relatively the same. To avoid page refreshes we should probably use a websocket to update the site.

### Multi-Boat Approach:

Currently, the codebase is hardcoded to show one voyage, but a goal of Zephyr is the ability to show multiple voyages at the same time for testing and the launch of autonamous sailboats simultaneously.

To do this we can explore a design that shows the cursors for all boats at the same time on the map, and different colors for their voyages. Clicking a voyage or toggling filters could let users view only one or a few of the boats. To display their data at the same time we can update ```Map.tsx``` and its components to batch query for the boat data and render it all.

### Database:

For the livestreaming and a multi-boat approach we will have frequent reads/writes to the database. To make reads efficient we can add indexes and query for the latest documents that we need. For example, we can bulk read documents when the map first loads to render the voyage paths, then on each clock tick (hourly, 30 minutes, etc.) we only fetch the X latest data points that we need.

If we are going to have multiple voyages, it would help to have unique identifiers for the documents so that we know which boat they belong to. Because we likely won't need to do many joins or complex queries, MongoDB is a good choice, especially because the data within documents may change and we can still query them without a migration.
