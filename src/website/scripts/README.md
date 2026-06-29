# aisstream-ingest.js

Standalone worker that streams live AIS vessel traffic from [aisstream.io](https://aisstream.io) and writes periodic snapshots to MongoDB for the website map. Run from `src/website` with `node scripts/aisstream-ingest.js`. Requires `MONGODB_URI` and `AISSTREAM_API_KEY`; exits immediately if either is missing.

Env files are loaded in order `.env.development` → `.env.local` → `.env`; for each variable, the **first** file that defines it wins (dotenv never overwrites).

## Pipeline

```
aisstream.io ──WebSocket──▶ ingest worker ──15 s flush──▶ MongoDB ──HTTP poll──▶ Redux store ──▶ Maps.tsx
                            (in-memory map)               (aisships)  (15 s)
```

1. **Subscribe.** On WebSocket open, the worker sends a subscription with the current bounding box(es), message-type filter (position reports + `ShipStaticData` by default), and optional MMSI filter.
2. **Accumulate.** Each position report upserts one in-memory entry per MMSI (latest position only). `ShipStaticData` messages populate a separate dimensions map (length = A+B, width = C+D from antenna offsets).
3. **Flush.** Every `AISSTREAM_FLUSH_INTERVAL_MS` (default **15 s**): re-evaluate the dynamic bbox, prune stale vessels, then write one snapshot document (`{ships: [...], timestamp}`) to the `AISShips` collection. The bbox check runs *before* the flush so a relocation is reflected in that same snapshot.
4. **Trim.** After each write, the collection is trimmed to the newest `AISSTREAM_MAX_SNAPSHOTS` (default **50**) documents by `_id` order — at 15 s cadence that is ~12.5 min of history.
5. **Serve.** `GET /api/aisships?latest=true` returns only the newest snapshot (what the map poller uses); without the flag the route returns all retained snapshots (dataset downloads).
6. **Render.** The frontend saga polls every `NEXT_PUBLIC_POLLING_TIME_MS` (**15 s** in development) and replaces the store with the newest snapshot. `Maps.tsx` renders exactly that snapshot — there is no client-side filtering, merging, or aging.

## Dynamic bounding box

Enabled by default (`AISSTREAM_DYNAMIC_BBOX_ENABLED=true`). The box follows the boat's GPS, read from the `gps` collection (newest document by `timestamp`, then `_id`).

- **Size:** `AISSTREAM_BBOX_RADIUS_KM` (default **30 km**) in every direction — a square roughly **60 km × 60 km** centered on the boat. Longitude width is scaled by `cos(latitude)` so the real-world width stays constant (floored near the poles); latitude is clamped to ±90°. If the box crosses the antimeridian it is split into two boxes; if it would span ≥360° of longitude it covers the full range.
- **Seeding:** at startup the worker reads the latest GPS fix and centers the box on it before connecting.
- **Shifting:** the box re-centers only after the boat has moved ≥ `AISSTREAM_BBOX_MOVE_THRESHOLD_KM` (default **5 km**) from the current center, to avoid resubscription churn. Checks are throttled to `AISSTREAM_BBOX_CHECK_INTERVAL_MS` (default = flush interval, 15 s). A shift resubscribes over the *same* socket — no reconnect.
- **Staleness:** GPS fixes older than `AISSTREAM_GPS_STALE_MS` (default **3 min**) are ignored and the box holds position. The stale warning is logged once per distinct fix, not per check.

### When MongoDB has no usable GPS fix

If the `gps` collection is empty, the newest fix is stale, the record has invalid coordinates/timestamp, or the read fails:

- At startup: the worker logs a warning and falls back to the static `AISSTREAM_BOUNDING_BOXES` value — or **whole-world coverage** `[[[-90,-180],[90,180]]]` if that is unset or unparseable.
- The box stays *unseeded*, which bypasses the check throttle: every 15 s tick re-reads GPS, so the box locks onto the boat within one tick of the first fresh fix and resubscribes immediately.
- Mid-run GPS loss: the box silently keeps its last position (one warning per stale fix). It never reverts to the static fallback once seeded.

## Eviction

Two mechanisms remove vessels from the in-memory map; the website map follows automatically because it only ever renders the latest snapshot.

- **TTL eviction:** at every flush, vessels not heard from within `AISSTREAM_VESSEL_TTL_MS` (default **10 min**) are dropped. Staleness is measured by local receive time, not vessel-reported time (robust to bad ship clocks). 10 min is deliberate: anchored class-A ships report only every ~3 min and static data every ~6 min, so a shorter TTL would make ships flicker.
- **Spatial eviction:** when the bbox shifts, vessels farther than **1.5 × radius** (45 km at defaults) from the new center are evicted immediately, so snapshots switch areas without waiting out the TTL. The 1.5× margin keeps near-edge ships from flickering.
- Dimension entries are kept while a vessel is position-tracked and dropped only once it is both untracked and past TTL, since statics refresh far less often than positions.

## Expected delays

| Action | Map reflects it after | Why |
|---|---|---|
| Ship sends a position report | 0–30 s (typ. ~15 s) | up to 15 s until next flush + up to 15 s until next frontend poll |
| Ship stops transmitting / sails away | up to ~10.5 min | 10 min TTL + flush + poll |
| Boat moves ≥5 km (bbox shift) | old-area ships gone in ≤ ~30 s | GPS fix lands (~1/min in production) → next 15 s cycle shifts box and prunes *before* writing that snapshot → poll |
| New-area ships after a shift | seconds to minutes | resubscription is instant; first appearance depends on each vessel's AIS transmit rate (class A: 2–10 s underway, ~3 min anchored; class B slower) |
| Correct ship dimensions appear | up to ~6 min | `ShipStaticData` broadcasts every ~6 min; until then 30 m × 10 m defaults are used. Dimensions attach on the vessel's *next* position report after statics arrive |
| GPS feed dies | box freezes after 3 min | fixes older than `AISSTREAM_GPS_STALE_MS` are ignored |
| WebSocket drops | map freezes at last snapshot | flush timer stops while disconnected; reconnect backoff is 2 s doubling to a 30 s cap. In-memory vessels are kept and TTL-pruned at the first flush after reconnect |

## Rules

1. One entry per MMSI: a snapshot never contains two positions for the same vessel.
2. **Empty snapshots are suppressed except after a region change.** TTL evictions never produce an empty snapshot (the map keeps the last non-empty one to avoid flicker), but a bbox shift that evicts vessels is allowed to write one empty snapshot so the map clears when the boat enters a region with no ships.
3. The map is snapshot-driven: anything evicted from the worker's memory disappears from the map at the next flush + poll; nothing is evicted client-side.
4. Missing `cog`/`rot`/`sog` are stored as `0`; missing dimensions as 30 m × 10 m.
5. The bbox check always runs before the flush in the same cycle, so shift + spatial eviction land in that cycle's snapshot, not the next.
6. Cycles never overlap: a slow flush skips the next timer tick rather than stacking.
7. Snapshot trimming orders by `_id` (monotonic), not the string `timestamp` field.
8. On `SIGINT`/`SIGTERM` the worker writes one final snapshot, then disconnects cleanly.
9. A stale/missing GPS fix never moves the box; it only ever holds position or stays on the fallback.

## Environment variables

| Variable | Default | Purpose |
|---|---|---|
| `MONGODB_URI` | — (required) | MongoDB connection string |
| `AISSTREAM_API_KEY` | — (required) | aisstream.io API key |
| `AISSTREAM_BOUNDING_BOXES` | whole world | Static fallback box(es), JSON `[[[S,W],[N,E]], ...]` |
| `AISSTREAM_DYNAMIC_BBOX_ENABLED` | `true` | GPS-following bbox on/off |
| `AISSTREAM_BBOX_RADIUS_KM` | `30` | Half-width of the dynamic box |
| `AISSTREAM_BBOX_MOVE_THRESHOLD_KM` | `5` | Boat movement required before shifting |
| `AISSTREAM_BBOX_CHECK_INTERVAL_MS` | flush interval | Throttle on bbox re-evaluation |
| `AISSTREAM_GPS_STALE_MS` | `180000` (3 min) | Max GPS fix age before it is ignored |
| `AISSTREAM_VESSEL_TTL_MS` | `600000` (10 min) | Vessel eviction TTL |
| `AISSTREAM_FILTER_SHIP_MMSI` | unset | Comma-separated MMSI allowlist |
| `AISSTREAM_FILTER_MESSAGE_TYPES` | position + static types | Comma-separated message-type filter |
| `AISSTREAM_FLUSH_INTERVAL_MS` | `15000` | Snapshot write cadence |
| `AISSTREAM_MAX_SNAPSHOTS` | `50` | Snapshot documents retained |
