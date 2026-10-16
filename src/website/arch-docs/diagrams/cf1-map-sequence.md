<!-- markdownlint-disable MD013 -->

# CF1: View the voyage on the map

A visitor opens the site, looks around the map, and clicks on a ship and on the boat. See "Core functionality" in [`ARCHITECTURE.md`](../ARCHITECTURE.md).

```mermaid
sequenceDiagram
    autonumber
    actor V as Visitor
    participant B as Browser<br/>(React + Redux)
    participant S as Next.js server<br/>(page + API routes)
    participant DB as MongoDB
    participant T as CARTO<br/>(map tiles)

    V->>B: Open the site (/)
    B->>S: GET /
    S-->>B: HTML shell + JavaScript (no data yet)
    Note over B: Redux store is created and starts<br/>polling loops for all 9 datasets

    B->>B: Draw map with the hardcoded<br/>voyage path and last known position
    B->>T: Request tiles for the visible area
    T-->>B: Map tile images

    loop Every polling interval (default 1 h in production)
        par AIS ships
            B->>S: GET /api/aisships?latest=true
            S->>DB: Find the newest snapshot in ais_ships
            DB-->>S: One snapshot document
            S-->>B: JSON
            B->>B: Save to Redux store, draw ship shapes
        and GPS, paths, and sensors (8 loops)
            B->>S: GET /api/gps, /api/localpath, ...
            S->>DB: Find the whole collection
            DB-->>S: Every document
            S-->>B: JSON
            Note over B: Saved to the store, but not drawn:<br/>the map uses hardcoded GPS and<br/>path/sensor views are hidden
        end
    end

    V->>B: Pan or zoom the map
    B->>T: Request tiles for the new area
    T-->>B: Map tile images

    V->>B: Click a ship
    B-->>V: Popup with the ship's details (from the store)

    V->>B: Click the boat marker
    B-->>V: Popup with the last known position (hardcoded)
```

## Takeaways

- **The server sends no data with the page.** Everything arrives afterwards, through polling loops in the browser.
- **Every loop starts at once and runs on the same timer,** including the 8 whose data the map doesn't draw today. Most loops fetch the **entire collection** every time. Only AIS asks for the latest snapshot.
- **Panning, zooming, and clicking never reach our backend.** Tiles come from CARTO, and popups use data already in the browser.
- **The boat's position and path are hardcoded** since the voyage ended. While the voyage was live, they came from the GPS loop.
