/**
 * AIS Stream ingest worker.
 * - Connects to aisstream.io via WebSocket.
 * - Listens for ship position/static messages.
 * - Keeps the latest position per vessel in memory and periodically writes a snapshot to MongoDB.
 * - Trims old snapshots to avoid unbounded growth.
 *
 * Run from src/website: `node scripts/aisstream-ingest.js`
 *
 * Required env:
 *   MONGODB_URI
 *   AISSTREAM_API_KEY
 *
 * Optional env:
 *   AISSTREAM_BOUNDING_BOXES          JSON string of bounding boxes e.g. [[[-90,-180],[90,180]]]
 *   AISSTREAM_DYNAMIC_BBOX_ENABLED    true/false to enable GPS-following bounding boxes (default: true)
 *   AISSTREAM_BBOX_RADIUS_KM          Radius in km around boat GPS used to build dynamic bbox (default: 30)
 *   AISSTREAM_BBOX_MOVE_THRESHOLD_KM  Minimum boat movement in km before bbox is shifted (default: 5)
 *   AISSTREAM_BBOX_CHECK_INTERVAL_MS  How often to re-evaluate the GPS-following bbox (default: flush interval)
 *   AISSTREAM_GPS_STALE_MS            Max allowed age of latest GPS fix in ms before ignoring it (default: 180000)
 *   AISSTREAM_VESSEL_TTL_MS           Max age of a vessel's last message before it is evicted (default: 600000)
 *   AISSTREAM_FILTER_SHIP_MMSI        Comma-separated MMSI list
 *   AISSTREAM_FILTER_MESSAGE_TYPES    Comma-separated list; defaults to common position/static messages
 *   AISSTREAM_FLUSH_INTERVAL_MS       Defaults to 15000
 *   AISSTREAM_MAX_SNAPSHOTS           Defaults to 50
 */

/* eslint-disable no-console */
const fs = require('fs');
const path = require('path');
const mongoose = require('mongoose');
const dotenv = require('dotenv');
const WebSocket = require('ws');

// dotenv does not overwrite keys that are already set, so for each variable
// the first file in this list that defines it wins.
const envCandidates = [
  path.resolve(__dirname, '../.env.development'),
  path.resolve(__dirname, '../.env.local'),
  path.resolve(__dirname, '../.env'),
];
envCandidates.forEach((envPath) => {
  if (fs.existsSync(envPath)) {
    dotenv.config({ path: envPath });
  }
});

const {
  MONGODB_URI,
  AISSTREAM_API_KEY,
  AISSTREAM_BOUNDING_BOXES,
  AISSTREAM_DYNAMIC_BBOX_ENABLED,
  AISSTREAM_BBOX_RADIUS_KM,
  AISSTREAM_BBOX_MOVE_THRESHOLD_KM,
  AISSTREAM_BBOX_CHECK_INTERVAL_MS,
  AISSTREAM_GPS_STALE_MS,
  AISSTREAM_VESSEL_TTL_MS,
  AISSTREAM_FILTER_SHIP_MMSI,
  AISSTREAM_FILTER_MESSAGE_TYPES,
  AISSTREAM_FLUSH_INTERVAL_MS,
  AISSTREAM_MAX_SNAPSHOTS,
} = process.env;

if (!MONGODB_URI) {
  console.error('Missing MONGODB_URI env var');
  process.exit(1);
}
if (!AISSTREAM_API_KEY) {
  console.error('Missing AISSTREAM_API_KEY env var');
  process.exit(1);
}

const DEFAULT_DIMENSIONS = { length: 30, width: 10 };
const WORLD_BOUNDING_BOXES = [[[-90, -180], [90, 180]]];
const EARTH_RADIUS_KM = 6371;

const POSITION_REPORT_TYPES = [
  'PositionReport',
  'StandardClassBPositionReport',
  'ExtendedClassBPositionReport',
];
const SUBSCRIBE_MESSAGE_TYPES = [...POSITION_REPORT_TYPES, 'ShipStaticData'];

const parseBoolean = (raw, defaultValue) => {
  if (raw === undefined || raw === null || String(raw).trim() === '') {
    return defaultValue;
  }
  return ['true', '1', 'yes', 'on'].includes(String(raw).trim().toLowerCase());
};

const parsePositiveNumber = (raw, fallback) => {
  const parsed = Number(raw);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
};

const DEFAULT_FLUSH_INTERVAL_MS = parsePositiveNumber(
  AISSTREAM_FLUSH_INTERVAL_MS,
  15000,
);
const MAX_SNAPSHOTS = parsePositiveNumber(AISSTREAM_MAX_SNAPSHOTS, 50);
const DYNAMIC_BBOX_ENABLED = parseBoolean(AISSTREAM_DYNAMIC_BBOX_ENABLED, true);
const BBOX_RADIUS_KM = parsePositiveNumber(AISSTREAM_BBOX_RADIUS_KM, 30);
// On a bbox shift, vessels farther than this multiple of the radius from the
// new center are evicted immediately, so snapshots switch areas without
// waiting out the TTL; the margin keeps near-edge ships from flickering.
const BBOX_PRUNE_RADIUS_FACTOR = 1.5;
// Minimum boat movement before shifting the bbox, to avoid resubscription churn.
const BBOX_MOVE_THRESHOLD_KM = parsePositiveNumber(
  AISSTREAM_BBOX_MOVE_THRESHOLD_KM,
  5,
);
// Defaults to the flush cadence so a relocated boat is picked up within one
// tick; the extra GPS read per tick is a trivial indexed findOne. Raise via
// env if that ever matters.
const BBOX_CHECK_INTERVAL_MS = parsePositiveNumber(
  AISSTREAM_BBOX_CHECK_INTERVAL_MS,
  DEFAULT_FLUSH_INTERVAL_MS,
);
// Ignore GPS fixes older than this so stale records cannot move the bbox.
const GPS_STALE_MS = parsePositiveNumber(AISSTREAM_GPS_STALE_MS, 180000);
// AIS gaps can reach ~3min (anchored) and static refresh ~6min, so 10min
// avoids flicker while still shedding ships that have left the area.
const VESSEL_TTL_MS = parsePositiveNumber(AISSTREAM_VESSEL_TTL_MS, 600000);

const parseBoundingBoxes = (raw) => {
  if (!raw) {
    return WORLD_BOUNDING_BOXES;
  }
  try {
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed) && parsed.length > 0) {
      return parsed;
    }
  } catch (err) {
    console.warn('Could not parse AISSTREAM_BOUNDING_BOXES, using world coverage.', err);
  }
  return WORLD_BOUNDING_BOXES;
};

const parseList = (raw) =>
  raw
    ? raw
        .split(',')
        .map((item) => item.trim())
        .filter(Boolean)
    : [];

const staticBoundingBoxes = parseBoundingBoxes(AISSTREAM_BOUNDING_BOXES);
const mmsiFilter = parseList(AISSTREAM_FILTER_SHIP_MMSI);
const configuredMessageTypes = parseList(AISSTREAM_FILTER_MESSAGE_TYPES);
const messageTypeFilter =
  configuredMessageTypes.length > 0
    ? configuredMessageTypes
    : SUBSCRIBE_MESSAGE_TYPES;

// Dynamic bbox state. The static boxes act as the fallback until (and unless)
// a fresh GPS fix seeds a boat-centered box.
let currentBoundingBoxes = staticBoundingBoxes;
let lastBoundingBoxCenter = null;
let lastBoundingBoxCheckMs = 0;
// Timestamp of the last GPS fix we warned about being stale, so a dead GPS
// feed is logged once per fix instead of on every check.
let lastLoggedStaleGpsMs = null;

const buildSubscriptionPayload = () => {
  const payload = {
    APIKey: AISSTREAM_API_KEY,
    BoundingBoxes: currentBoundingBoxes,
    FilterMessageTypes: messageTypeFilter,
  };
  if (mmsiFilter.length > 0) {
    payload.FiltersShipMMSI = mmsiFilter;
  }
  return payload;
};

const vesselPositions = new Map(); // MMSI -> latest position
const vesselStatics = new Map(); // MMSI -> { length, width, receivedAt }

const AISShipsSchema = new mongoose.Schema({
  ships: {
    type: [
      {
        id: Number,
        latitude: mongoose.Schema.Types.Decimal128,
        longitude: mongoose.Schema.Types.Decimal128,
        cog: mongoose.Schema.Types.Decimal128,
        rot: mongoose.Schema.Types.Decimal128,
        sog: mongoose.Schema.Types.Decimal128,
        width: mongoose.Schema.Types.Decimal128,
        length: mongoose.Schema.Types.Decimal128,
      },
    ],
    required: [true, 'Missing array of AIS ships'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

const AISShips =
  mongoose.models.AISShips || mongoose.model('AISShips', AISShipsSchema);

// Empty strings must be rejected explicitly because Number('') === 0.
const toNumber = (value) => {
  if (value === undefined || value === null || value === '') {
    return undefined;
  }
  const parsed = Number(value);
  return Number.isNaN(parsed) ? undefined : parsed;
};

const clampLatitude = (latitude) => Math.max(-90, Math.min(90, latitude));

// Wrap a longitude into the -180..180 range expected by AISstream.
const normalizeLongitude = (longitude) => {
  let normalized = longitude;
  while (normalized > 180) {
    normalized -= 360;
  }
  while (normalized < -180) {
    normalized += 360;
  }
  return normalized;
};

const toRadians = (degrees) => (degrees * Math.PI) / 180;

// Great-circle distance in km between two positions.
const haversineKm = (lat1, lon1, lat2, lon2) => {
  const deltaLat = toRadians(lat2 - lat1);
  const deltaLon = toRadians(lon2 - lon1);
  const a =
    Math.sin(deltaLat / 2) ** 2 +
    Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) * Math.sin(deltaLon / 2) ** 2;
  return 2 * EARTH_RADIUS_KM * Math.asin(Math.sqrt(a));
};

// Build AIS bbox(es) from a center point + radius in km. Splits into two boxes
// when the window crosses the antimeridian (±180°) so each box keeps
// west <= east, which is what AISstream requires.
const buildBoundingBoxesAroundCenter = (latitude, longitude, radiusKm) => {
  const latitudeDelta = radiusKm / 111;
  // Longitude degrees shrink with latitude; the 0.1 floor keeps the box finite
  // near the poles.
  const longitudeScale = Math.max(0.1, Math.cos(toRadians(latitude)));
  const longitudeDelta = radiusKm / (111 * longitudeScale);

  const south = clampLatitude(latitude - latitudeDelta);
  const north = clampLatitude(latitude + latitudeDelta);

  // A window spanning >= 360 degrees wraps the whole globe; splitting it would
  // leave a gap, so cover the full longitude range instead.
  if (longitudeDelta >= 180) {
    return [[[south, -180], [north, 180]]];
  }

  const rawWest = longitude - longitudeDelta;
  const rawEast = longitude + longitudeDelta;

  if (rawEast > 180 || rawWest < -180) {
    return [
      [[south, normalizeLongitude(rawWest)], [north, 180]],
      [[south, -180], [north, normalizeLongitude(rawEast)]],
    ];
  }

  return [[[south, rawWest], [north, rawEast]]];
};

const parseTimestampMs = (timestamp) => {
  const parsedMs = Date.parse(timestamp);
  return Number.isNaN(parsedMs) ? undefined : parsedMs;
};

// Read the latest boat GPS fix, or null when there is no usable (fresh, valid)
// fix — in which case the current bbox is silently kept.
const readLatestBoatGps = async () => {
  try {
    const latest = await mongoose.connection.collection('gps').findOne(
      {},
      {
        sort: { timestamp: -1, _id: -1 },
        projection: { latitude: 1, longitude: 1, timestamp: 1 },
      },
    );

    if (!latest) {
      return null;
    }

    // Coordinates may be stored as Decimal128, so stringify before converting.
    const latitude = toNumber(String(latest.latitude));
    const longitude = toNumber(String(latest.longitude));

    if (latitude === undefined || longitude === undefined) {
      console.warn('Latest GPS record has invalid latitude/longitude; skipping bbox update.');
      return null;
    }

    const timestampMs = parseTimestampMs(latest.timestamp);

    if (timestampMs === undefined) {
      console.warn('Latest GPS record has invalid timestamp; skipping bbox update.');
      return null;
    }

    if (Date.now() - timestampMs > GPS_STALE_MS) {
      if (lastLoggedStaleGpsMs !== timestampMs) {
        lastLoggedStaleGpsMs = timestampMs;
        console.warn(
          `Newest GPS fix (${new Date(timestampMs).toISOString()}) is older than ${GPS_STALE_MS} ms; dynamic bbox holding position.`,
        );
      }
      return null;
    }

    return {
      latitude,
      longitude,
      timestamp: latest.timestamp,
    };
  } catch (err) {
    console.warn('Failed reading latest boat GPS.', err);
    return null;
  }
};

// Evict tracked vessels that are no longer near a given center so snapshots
// switch to the new area immediately instead of waiting out VESSEL_TTL_MS.
const pruneVesselsOutsideRadius = (center, radiusKm) => {
  let removed = 0;
  for (const [mmsi, ship] of vesselPositions) {
    const distanceKm = haversineKm(
      center.latitude,
      center.longitude,
      ship.latitude,
      ship.longitude,
    );
    if (distanceKm > radiusKm) {
      vesselPositions.delete(mmsi);
      removed += 1;
    }
  }
  return removed;
};

const formatEvictionNote = (count) =>
  count > 0 ? ` (evicted ${count} out-of-area vessel(s))` : '';

// Re-center the dynamic bbox on a fresh GPS fix and (if connected) resubscribe.
// Shared by startup, first-fix seeding, and movement-triggered shifts so the
// bbox state transitions live in exactly one place. Returns how many
// out-of-area vessels were evicted.
const applyDynamicBoundingBox = (latestGps, socket) => {
  currentBoundingBoxes = buildBoundingBoxesAroundCenter(
    latestGps.latitude,
    latestGps.longitude,
    BBOX_RADIUS_KM,
  );
  lastBoundingBoxCenter = {
    latitude: latestGps.latitude,
    longitude: latestGps.longitude,
  };
  // Applying a box counts as an evaluation, so the next throttled check waits
  // a full interval.
  lastBoundingBoxCheckMs = Date.now();
  const pruned = pruneVesselsOutsideRadius(
    lastBoundingBoxCenter,
    BBOX_RADIUS_KM * BBOX_PRUNE_RADIUS_FACTOR,
  );
  // Resubscribe over the SAME socket (no reconnect) so aisstream switches region.
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(buildSubscriptionPayload()));
  }
  return pruned;
};

const initializeBoundingBoxesFromBoatGps = async () => {
  if (!DYNAMIC_BBOX_ENABLED) {
    console.log('Dynamic AIS bbox disabled; using static AISSTREAM_BOUNDING_BOXES value.');
    return;
  }

  const latestGps = await readLatestBoatGps();
  if (!latestGps) {
    console.warn('Could not initialize dynamic bbox from GPS; using static fallback bounding box.');
    return;
  }

  // No socket exists yet at startup; onopen sends the subscription.
  applyDynamicBoundingBox(latestGps, null);
  console.log(
    `Initialized dynamic AIS bbox from boat GPS (radius ${BBOX_RADIUS_KM} km): ${JSON.stringify(currentBoundingBoxes)}`,
  );
};

// Re-evaluate the dynamic bbox against the latest GPS fix: seed it on the first
// valid fix, otherwise shift it only once the boat has moved past the threshold.
const maybeUpdateDynamicBoundingBoxes = async (socket) => {
  if (!DYNAMIC_BBOX_ENABLED) {
    return;
  }
  const latestGps = await readLatestBoatGps();
  if (!latestGps) {
    return;
  }

  if (!lastBoundingBoxCenter) {
    const pruned = applyDynamicBoundingBox(latestGps, socket);
    console.log(
      `Seeded dynamic AIS bbox from boat GPS${formatEvictionNote(pruned)}: ${JSON.stringify(currentBoundingBoxes)}`,
    );
    return;
  }

  const movedKm = haversineKm(
    lastBoundingBoxCenter.latitude,
    lastBoundingBoxCenter.longitude,
    latestGps.latitude,
    latestGps.longitude,
  );
  if (movedKm < BBOX_MOVE_THRESHOLD_KM) {
    return;
  }

  const pruned = applyDynamicBoundingBox(latestGps, socket);
  console.log(
    `Shifted AIS bbox after ${movedKm.toFixed(1)} km of movement${formatEvictionNote(pruned)}: ${JSON.stringify(currentBoundingBoxes)}`,
  );
};

const extractDimensions = (shipStaticData = {}) => {
  const dimension = shipStaticData.Dimension || shipStaticData.dimension;
  if (!dimension) {
    return null;
  }
  // AIS reports dimensions as distances from the GPS antenna:
  // A = to bow, B = to stern, C = to port, D = to starboard.
  const bow = toNumber(dimension.A);
  const stern = toNumber(dimension.B);
  const port = toNumber(dimension.C);
  const starboard = toNumber(dimension.D);

  const length =
    bow !== undefined && stern !== undefined ? bow + stern : undefined;
  const width =
    port !== undefined && starboard !== undefined ? port + starboard : undefined;

  if (length === undefined && width === undefined) {
    return null;
  }

  return {
    length: length ?? DEFAULT_DIMENSIONS.length,
    width: width ?? DEFAULT_DIMENSIONS.width,
  };
};

const normalizePosition = (aisMessage) => {
  const meta = aisMessage.MetaData || aisMessage.metadata || {};
  const messageBody = aisMessage.Message || aisMessage.message || {};

  const positionPayload =
    messageBody.PositionReport ||
    messageBody.StandardClassBPositionReport ||
    messageBody.ExtendedClassBPositionReport ||
    null;

  if (!positionPayload) {
    return null;
  }

  const mmsi =
    meta.MMSI ||
    meta.mmsi ||
    positionPayload.UserID ||
    positionPayload.MMSI ||
    positionPayload.UserId;

  if (!mmsi) {
    return null;
  }

  const latitude =
    toNumber(meta.latitude ?? meta.Latitude ?? positionPayload.Latitude) ??
    null;
  const longitude =
    toNumber(meta.longitude ?? meta.Longitude ?? positionPayload.Longitude) ??
    null;
  const cog = toNumber(positionPayload.Cog ?? positionPayload.CourseOverGround);
  const sog = toNumber(positionPayload.Sog ?? positionPayload.SpeedOverGround);
  const rot = toNumber(positionPayload.RateOfTurn ?? positionPayload.Rot);

  const staticDims = vesselStatics.get(String(mmsi)) || {};

  return {
    id: Number(mmsi),
    latitude,
    longitude,
    cog,
    sog,
    rot,
    width: staticDims.width ?? DEFAULT_DIMENSIONS.width,
    length: staticDims.length ?? DEFAULT_DIMENSIONS.length,
    updatedAt: meta.time_utc || new Date().toISOString(),
    // Local receive time drives staleness eviction (robust to bad vessel clocks).
    receivedAt: Date.now(),
  };
};

const handleStaticData = (aisMessage) => {
  const meta = aisMessage.MetaData || aisMessage.metadata || {};
  const messageBody = aisMessage.Message || aisMessage.message || {};
  const shipStatic = messageBody.ShipStaticData || messageBody.ShipStatic;
  if (!shipStatic) {
    return;
  }
  const mmsi = meta.MMSI || meta.mmsi || shipStatic.UserID || shipStatic.MMSI;
  if (!mmsi) {
    return;
  }
  const dims = extractDimensions(shipStatic);
  if (dims) {
    vesselStatics.set(String(mmsi), { ...dims, receivedAt: Date.now() });
  }
};

// Normalize WebSocket payloads across Node (Buffer/Uint8Array) and undici (Blob).
const readPayload = async (rawData) => {
  if (typeof rawData === 'string') {
    return rawData;
  }
  if (rawData instanceof Buffer || rawData instanceof Uint8Array) {
    return rawData.toString('utf8');
  }
  if (rawData && typeof rawData.text === 'function') {
    return await rawData.text();
  }
  return String(rawData);
};

const processMessage = async (rawData) => {
  let parsed;
  try {
    const payload = await readPayload(rawData);
    parsed = JSON.parse(payload);
  } catch (err) {
    console.warn('Received non-JSON AIS message', err);
    return;
  }

  if (parsed.MessageType === 'ShipStaticData') {
    handleStaticData(parsed);
    return;
  }

  if (!POSITION_REPORT_TYPES.includes(parsed.MessageType)) {
    return;
  }

  const position = normalizePosition(parsed);
  if (!position || position.latitude === null || position.longitude === null) {
    return;
  }

  vesselPositions.set(String(position.id), position);
};

// Evict vessels not heard from within VESSEL_TTL_MS so both Maps stay bounded
// and snapshots only contain ships currently near the boat.
const pruneStaleVessels = (now = Date.now()) => {
  let removed = 0;
  for (const [mmsi, ship] of vesselPositions) {
    if (now - ship.receivedAt > VESSEL_TTL_MS) {
      vesselPositions.delete(mmsi);
      removed += 1;
    }
  }
  // Keep dimensions for actively-tracked vessels even when their static entry
  // is old, because static data refreshes far less often than positions.
  for (const [mmsi, dims] of vesselStatics) {
    if (!vesselPositions.has(mmsi) && now - dims.receivedAt > VESSEL_TTL_MS) {
      vesselStatics.delete(mmsi);
    }
  }
  if (removed > 0) {
    console.log(`Evicted ${removed} stale vessel(s) (TTL ${(VESSEL_TTL_MS / 60000).toFixed(1)} min).`);
  }
};

// Keep the snapshot collection bounded by deleting the oldest documents.
// Oldest is determined by _id order (monotonic and default-indexed) rather
// than the string timestamp field.
const trimSnapshots = async () => {
  try {
    // The newest document past the retention window marks the cutoff;
    // everything at or below it is excess.
    const cutoff = await AISShips.findOne({})
      .sort({ _id: -1 })
      .skip(MAX_SNAPSHOTS)
      .select('_id');
    if (cutoff) {
      await AISShips.deleteMany({ _id: { $lte: cutoff._id } });
    }
  } catch (err) {
    console.error('Failed trimming AIS snapshots', err);
  }
};

// Persist the latest known position of every tracked vessel as one snapshot.
const flushSnapshot = async () => {
  pruneStaleVessels();
  if (vesselPositions.size === 0) {
    return;
  }
  const ships = Array.from(vesselPositions.values()).map((ship) => ({
    id: ship.id,
    latitude: ship.latitude,
    longitude: ship.longitude,
    cog: ship.cog ?? 0,
    rot: ship.rot ?? 0,
    sog: ship.sog ?? 0,
    width: ship.width,
    length: ship.length,
  }));

  try {
    await AISShips.create({
      ships,
      timestamp: new Date().toISOString(),
    });
    await trimSnapshots();
    console.log(`Saved AIS snapshot with ${ships.length} ships.`);
  } catch (err) {
    console.error('Failed to write AIS snapshot', err);
  }
};

let reconnectAttempts = 0;
let flushIntervalId = null;
// Guards against a slow flush/update cycle overlapping the next timer tick.
let cycleInProgress = false;
// Declared up-front so scheduleReconnect can read it before main() runs.
let shuttingDown = false;

const runPeriodicCycle = async (socket) => {
  if (cycleInProgress) {
    return;
  }
  cycleInProgress = true;
  try {
    // Check the bbox before flushing so a relocation (shift + out-of-area
    // eviction) is reflected in this cycle's snapshot rather than the next.
    // Throttled to BBOX_CHECK_INTERVAL_MS, except while the box is still
    // unseeded (e.g. GPS was missing at startup) — then check every tick so
    // we lock onto the boat the moment a fix arrives.
    const now = Date.now();
    const boxSeeded = lastBoundingBoxCenter !== null;
    if (!boxSeeded || now - lastBoundingBoxCheckMs >= BBOX_CHECK_INTERVAL_MS) {
      lastBoundingBoxCheckMs = now;
      await maybeUpdateDynamicBoundingBoxes(socket);
    }
    await flushSnapshot();
  } catch (err) {
    // Keep the timer alive if one cycle fails.
    console.error('Periodic flush/update cycle failed.', err);
  } finally {
    cycleInProgress = false;
  }
};

const startWebSocket = () => {
  console.log('Connecting to aisstream.io ...');
  const socket = new WebSocket('wss://stream.aisstream.io/v0/stream');

  socket.onopen = () => {
    reconnectAttempts = 0;
    try {
      // Build the payload at send-time so current bbox state is always used.
      socket.send(JSON.stringify(buildSubscriptionPayload()));
      console.log('Subscription sent to aisstream.io');
    } catch (err) {
      console.error('Failed to send subscription message', err);
    }

    if (flushIntervalId) {
      clearInterval(flushIntervalId);
    }
    flushIntervalId = setInterval(
      () => runPeriodicCycle(socket),
      DEFAULT_FLUSH_INTERVAL_MS,
    );
  };

  socket.onmessage = (event) => {
    processMessage(event.data).catch((err) => {
      console.error('Failed processing AIS message', err);
    });
  };

  socket.onerror = (err) => {
    console.error('WebSocket error', err);
  };

  socket.onclose = (event) => {
    console.warn(
      `WebSocket closed (code ${event.code}, reason: ${event.reason}). Reconnecting...`,
    );
    if (flushIntervalId) {
      clearInterval(flushIntervalId);
      flushIntervalId = null;
    }
    // Detach handlers so the closed socket and its closures are freed promptly
    // rather than lingering across reconnects.
    socket.onopen = null;
    socket.onmessage = null;
    socket.onerror = null;
    socket.onclose = null;
    scheduleReconnect();
  };
};

const scheduleReconnect = () => {
  if (shuttingDown) {
    return;
  }
  reconnectAttempts += 1;
  const backoff = Math.min(30000, 1000 * 2 ** reconnectAttempts);
  setTimeout(startWebSocket, backoff);
};

const main = async () => {
  console.log('Starting AIS ingest worker');
  await mongoose.connect(MONGODB_URI);
  console.log('Connected to MongoDB');
  // Dynamic bbox initialization needs the DB connection for GPS queries.
  await initializeBoundingBoxesFromBoatGps();
  startWebSocket();
};

// Flush a final snapshot and disconnect cleanly on termination. Containers
// send SIGTERM (not SIGINT) on stop/redeploy, so handle both.
const shutdown = async (signal) => {
  if (shuttingDown) {
    return;
  }
  shuttingDown = true;
  console.log(`Received ${signal}; shutting down...`);
  if (flushIntervalId) {
    clearInterval(flushIntervalId);
    flushIntervalId = null;
  }
  try {
    await flushSnapshot();
    await mongoose.disconnect();
  } catch (err) {
    console.error('Error during shutdown.', err);
  }
  process.exit(0);
};

process.on('SIGINT', () => shutdown('SIGINT'));
process.on('SIGTERM', () => shutdown('SIGTERM'));

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
