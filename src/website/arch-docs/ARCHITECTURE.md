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
