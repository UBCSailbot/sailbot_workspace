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
 *   AISSTREAM_GPS_STALE_MS            Max allowed age of latest GPS fix in ms before ignoring it (default: 180000)
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

const envCandidates = [
  path.resolve(__dirname, '../.env.development'),
  path.resolve(__dirname, '../.env.local'),
  path.resolve(__dirname, '../.env'),
];
envCandidates.forEach((envPath) => {
  // Load the first env file we find so local dev stays simple.
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
  AISSTREAM_GPS_STALE_MS,
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

const DEFAULT_FLUSH_INTERVAL_MS = Number(AISSTREAM_FLUSH_INTERVAL_MS) || 15000;
const MAX_SNAPSHOTS = Number(AISSTREAM_MAX_SNAPSHOTS) || 50;
const DEFAULT_DIMENSIONS = { length: 30, width: 10 };
// Keep one shared "world" bbox fallback value so all fallback paths stay consistent.
const WORLD_BOUNDING_BOXES = [[[-90, -180], [90, 180]]];
// Use Earth radius in km for distance calculations when checking boat movement threshold.
const EARTH_RADIUS_KM = 6371;
const POSITION_MESSAGE_TYPES = [
  'PositionReport',
  'StandardClassBPositionReport',
  'ExtendedClassBPositionReport',
  'ShipStaticData',
];

// Parse boolean-like env values safely while preserving a default for missing values.
const parseBoolean = (raw, defaultValue) => {
  // If the env var is not provided, return the supplied default behavior.
  if (raw === undefined || raw === null || String(raw).trim() === '') {
    return defaultValue;
  }
  // Normalize common truthy strings so runtime config is forgiving.
  return ['true', '1', 'yes', 'on'].includes(String(raw).trim().toLowerCase());
};

// Parse positive numeric env values and fallback if parsing fails.
const parsePositiveNumber = (raw, fallback) => {
  // Convert once so we can validate numeric correctness.
  const parsed = Number(raw);
  // Only accept finite positive values; otherwise use the fallback.
  return Number.isFinite(parsed) && parsed > 0 ? parsed : fallback;
};

// Toggle dynamic bbox behavior (GPS-following) with safe default enabled.
const DYNAMIC_BBOX_ENABLED = parseBoolean(AISSTREAM_DYNAMIC_BBOX_ENABLED, true);
// Radius used to convert latest boat GPS into a bounding box window.
const BBOX_RADIUS_KM = parsePositiveNumber(AISSTREAM_BBOX_RADIUS_KM, 30);
// Movement threshold used to avoid excessive resubscriptions when boat barely moves.
const BBOX_MOVE_THRESHOLD_KM = parsePositiveNumber(
  AISSTREAM_BBOX_MOVE_THRESHOLD_KM,
  5,
);
// Staleness guard so old GPS records do not drive subscription updates.
const GPS_STALE_MS = parsePositiveNumber(AISSTREAM_GPS_STALE_MS, 180000);

const parseBoundingBoxes = (raw) => {
  if (!raw) {
    // Fall back to whole-world coverage when no static bbox is provided.
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
  // Fall back to whole-world coverage if parsing fails.
  return WORLD_BOUNDING_BOXES;
};

const parseList = (raw) =>
  raw
    ? raw
        .split(',')
        .map((item) => item.trim())
        .filter(Boolean)
    : [];

// Parse static bounding boxes once so we can use them as a fallback path.
const staticBoundingBoxes = parseBoundingBoxes(AISSTREAM_BOUNDING_BOXES);
const mmsiFilter = parseList(AISSTREAM_FILTER_SHIP_MMSI);
const messageTypeFilter =
  parseList(AISSTREAM_FILTER_MESSAGE_TYPES).length > 0
    ? parseList(AISSTREAM_FILTER_MESSAGE_TYPES)
    : POSITION_MESSAGE_TYPES;

// Track the currently-active bbox set used when creating subscription payloads.
let currentBoundingBoxes = staticBoundingBoxes;
// Track the last boat center used to build bbox so we can detect movement in km.
let lastBoundingBoxCenter = null;

// Build an aisstream subscription payload from current runtime state.
const buildSubscriptionPayload = () => {
  // Always include API key + current bbox + message-type filter.
  const payload = {
    APIKey: AISSTREAM_API_KEY,
    BoundingBoxes: currentBoundingBoxes,
    FilterMessageTypes: messageTypeFilter,
  };
  // Include MMSI filter only when user configured one.
  if (mmsiFilter.length > 0) {
    payload.FiltersShipMMSI = mmsiFilter;
  }
  // Return fully assembled payload to caller.
  return payload;
};

const vesselPositions = new Map(); // MMSI -> latest position
const vesselStatics = new Map(); // MMSI -> { length, width }

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

const toNumber = (value) =>
  value === undefined || value === null || Number.isNaN(Number(value))
    ? undefined
    : Number(value);

// Clamp latitude into valid Earth bounds.
const clampLatitude = (latitude) => Math.max(-90, Math.min(90, latitude));

// Normalize longitude into the -180..180 range expected by AISstream bounding boxes.
const normalizeLongitude = (longitude) => {
  // Wrap longitude values repeatedly into the target interval.
  let normalized = longitude;
  while (normalized > 180) {
    normalized -= 360;
  }
  while (normalized < -180) {
    normalized += 360;
  }
  return normalized;
};

// Convert degrees to radians for trigonometric calculations.
const toRadians = (degrees) => (degrees * Math.PI) / 180;

// Compute great-circle distance in km between two positions for movement threshold checks.
const haversineKm = (lat1, lon1, lat2, lon2) => {
  const deltaLat = toRadians(lat2 - lat1);
  const deltaLon = toRadians(lon2 - lon1);
  const a =
    Math.sin(deltaLat / 2) ** 2 +
    Math.cos(toRadians(lat1)) * Math.cos(toRadians(lat2)) * Math.sin(deltaLon / 2) ** 2;
  return 2 * EARTH_RADIUS_KM * Math.asin(Math.sqrt(a));
};

// Build a single AIS bbox from center point + radius in km.
const buildBoundingBoxAroundCenter = (latitude, longitude, radiusKm) => {
  const latitudeDelta = radiusKm / 111;
  const longitudeScale = Math.max(0.1, Math.cos(toRadians(latitude)));
  const longitudeDelta = radiusKm / (111 * longitudeScale);
  const southWest = [
    clampLatitude(latitude - latitudeDelta),
    normalizeLongitude(longitude - longitudeDelta),
  ];
  const northEast = [
    clampLatitude(latitude + latitudeDelta),
    normalizeLongitude(longitude + longitudeDelta),
  ];
  return [southWest, northEast];
};

// Parse a timestamp string into epoch ms, returning undefined for invalid inputs.
const parseTimestampMs = (timestamp) => {
  // Parse timestamp using JS Date parser.
  const parsedMs = Date.parse(timestamp);
  // Reject NaN parse results.
  return Number.isNaN(parsedMs) ? undefined : parsedMs;
};

// Read latest boat GPS point from MongoDB for dynamic bbox updates.
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
      console.warn('No GPS records found; keeping current AIS bounding boxes.');
      return null;
    }

    const latitudeRaw =
      latest.latitude && typeof latest.latitude.toString === 'function'
        ? latest.latitude.toString()
        : latest.latitude;
    const longitudeRaw =
      latest.longitude && typeof latest.longitude.toString === 'function'
        ? latest.longitude.toString()
        : latest.longitude;

    const latitude = toNumber(latitudeRaw);
    const longitude = toNumber(longitudeRaw);

    if (latitude === undefined || longitude === undefined) {
      console.warn('Latest GPS record has invalid latitude/longitude; skipping bbox update.');
      return null;
    }

    const timestampMs = parseTimestampMs(latest.timestamp);

    if (timestampMs === undefined) {
      console.warn('Latest GPS record has invalid timestamp; skipping bbox update.');
      return null;
    }

    const ageMs = Date.now() - timestampMs;

    if (ageMs > GPS_STALE_MS) {
      console.warn(
        `Latest GPS fix is stale (${ageMs}ms old); keeping current AIS bounding boxes.`,
      );
      return null;
    }

    return {
      latitude,
      longitude,
      timestamp: latest.timestamp,
    };
  } catch (err) {
    console.warn('Failed reading latest boat GPS; keeping current AIS bounding boxes.', err);
    return null;
  }
};

// Initialize bbox state from GPS at startup when dynamic mode is enabled.
const initializeBoundingBoxesFromBoatGps = async () => {
  if (!DYNAMIC_BBOX_ENABLED) {
    console.log('Dynamic AIS bbox disabled; using static AISSTREAM_BOUNDING_BOXES value.');
    return;
  }

  // Read latest boat GPS so initial subscription starts near boat position.
  const latestGps = await readLatestBoatGps();

  // If no valid GPS is available, retain static fallback bbox.
  if (!latestGps) {
    console.warn('Could not initialize dynamic bbox from GPS; using static fallback bounding box.');
    return;
  }

  // Build bbox centered on latest boat GPS using configured radius.
  currentBoundingBoxes = [
    buildBoundingBoxAroundCenter(latestGps.latitude, latestGps.longitude, BBOX_RADIUS_KM),
  ];
  // Store center point used for threshold-based movement checks.
  lastBoundingBoxCenter = {
    latitude: latestGps.latitude,
    longitude: latestGps.longitude,
  };
  // Emit startup log so operators can confirm dynamic mode is active.
  console.log('Initialized dynamic AIS bounding box from latest boat GPS.', {
    currentBoundingBoxes,
    radiusKm: BBOX_RADIUS_KM,
  });
};

// Shift bbox when boat movement exceeds threshold and optionally resubscribe immediately.
const maybeUpdateDynamicBoundingBoxes = async (socket) => {
  if (!DYNAMIC_BBOX_ENABLED) {
    return;
  }
  const latestGps = await readLatestBoatGps();

  if (!latestGps) {
    return;
  }
  if (!lastBoundingBoxCenter) {
    currentBoundingBoxes = [
      buildBoundingBoxAroundCenter(latestGps.latitude, latestGps.longitude, BBOX_RADIUS_KM),
    ];
    lastBoundingBoxCenter = {
      latitude: latestGps.latitude,
      longitude: latestGps.longitude,
    };
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(buildSubscriptionPayload()));
      console.log('Seeded dynamic AIS bounding box and sent subscription update.');
    }
    return;
  }

  // Compute movement distance from last bbox center to latest GPS point.
  const movedKm = haversineKm(
    lastBoundingBoxCenter.latitude,
    lastBoundingBoxCenter.longitude,
    latestGps.latitude,
    latestGps.longitude,
  );

  // Log movement diagnostics for operations visibility.
  console.log(
    `Boat movement since last bbox center: ${movedKm.toFixed(3)} km (threshold: ${BBOX_MOVE_THRESHOLD_KM} km).`,
  );

  // Avoid bbox churn when movement is below configured threshold.
  if (movedKm < BBOX_MOVE_THRESHOLD_KM) {
    return;
  }

  // Recompute bbox centered on latest boat GPS.
  currentBoundingBoxes = [
    buildBoundingBoxAroundCenter(latestGps.latitude, latestGps.longitude, BBOX_RADIUS_KM),
  ];
  // Update center baseline after accepted movement.
  lastBoundingBoxCenter = {
    latitude: latestGps.latitude,
    longitude: latestGps.longitude,
  };

  // Log bbox update so operators can audit live window shifts.
  console.log('Updated dynamic AIS bounding box after movement threshold crossing.', {
    currentBoundingBoxes,
    movedKm,
  });

  // Send updated subscription immediately when websocket is available.
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(buildSubscriptionPayload()));
    console.log('Sent updated AIS subscription payload with shifted dynamic bounding box.');
  }
};

const extractDimensions = (shipStaticData = {}) => {
  const dimension = shipStaticData.Dimension || shipStaticData.dimension;
  if (!dimension) {
    return null;
  }
  const bow = toNumber(dimension.A);
  const stern = toNumber(dimension.B);
  const port = toNumber(dimension.C);
  const starboard = toNumber(dimension.D);

  const length =
    bow !== undefined && stern !== undefined
      ? bow + stern
      : undefined;
  const width =
    port !== undefined && starboard !== undefined
      ? port + starboard
      : undefined;

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
  const cog =
    toNumber(positionPayload.Cog ?? positionPayload.CourseOverGround) ??
    undefined;
  const sog =
    toNumber(positionPayload.Sog ?? positionPayload.SpeedOverGround) ??
    undefined;
  const rot =
    toNumber(positionPayload.RateOfTurn ?? positionPayload.Rot) ?? undefined;

  const staticDims = vesselStatics.get(String(mmsi)) || {};
  const length = staticDims.length ?? DEFAULT_DIMENSIONS.length;
  const width = staticDims.width ?? DEFAULT_DIMENSIONS.width;

  return {
    id: Number(mmsi),
    latitude,
    longitude,
    cog,
    sog,
    rot,
    width,
    length,
    updatedAt: meta.time_utc || new Date().toISOString(),
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
    vesselStatics.set(String(mmsi), dims);
  }
};

const readPayload = async (rawData) => {
  // Normalize WebSocket payloads across Node (Buffer/Uint8Array) and undici (Blob).
  if (typeof rawData === 'string') {
    return rawData;
  }
  // Node WebSocket may deliver Buffer/Uint8Array
  if (rawData instanceof Buffer || rawData instanceof Uint8Array) {
    return rawData.toString('utf8');
  }
  // Undici WebSocket can deliver Blob
  if (rawData && typeof rawData.text === 'function') {
    return await rawData.text();
  }
  // Fallback
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

  if (!parsed.MessageType) {
    return;
  }

  if (parsed.MessageType === 'ShipStaticData') {
    handleStaticData(parsed);
    return;
  }

  if (!POSITION_MESSAGE_TYPES.includes(parsed.MessageType)) {
    return;
  }

  const position = normalizePosition(parsed);
  if (!position || position.latitude === null || position.longitude === null) {
    return;
  }

  vesselPositions.set(String(position.id), position);
};

const flushSnapshot = async () => {
  if (vesselPositions.size === 0) {
    return;
  }
  // Persist the latest known positions for each vessel as a single snapshot.
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

const trimSnapshots = async () => {
  try {
    const total = await AISShips.countDocuments();
    if (total <= MAX_SNAPSHOTS) {
      return;
    }
    const excess = total - MAX_SNAPSHOTS;
    const oldest = await AISShips.find({})
      .sort({ timestamp: 1 })
      .limit(excess)
      .select('_id');
    const ids = oldest.map((doc) => doc._id);
    if (ids.length > 0) {
      await AISShips.deleteMany({ _id: { $in: ids } });
      console.log(`Trimmed ${ids.length} old AIS snapshots.`);
    }
  } catch (err) {
    console.error('Failed trimming AIS snapshots', err);
  }
};

let reconnectAttempts = 0;
let flushIntervalId = null;

const startWebSocket = () => {
  // Connect and keep a long-lived WebSocket alive with exponential backoff.
  console.log('Connecting to aisstream.io ...');
  const socket = new WebSocket('wss://stream.aisstream.io/v0/stream');

  socket.onopen = () => {
    reconnectAttempts = 0;
    try {
      // Build payload at send-time so current dynamic/static bbox state is always used.
      socket.send(JSON.stringify(buildSubscriptionPayload()));
      console.log('Subscription sent to aisstream.io');
    } catch (err) {
      console.error('Failed to send subscription message', err);
    }

    if (flushIntervalId) {
      clearInterval(flushIntervalId);
    }
    // Reuse existing flush timer to also run movement-threshold bbox checks.
    flushIntervalId = setInterval(async () => {
      try {
        // Persist latest AIS vessel snapshot into MongoDB.
        await flushSnapshot();
        // Evaluate dynamic bbox shift and resubscribe only when threshold is crossed.
        await maybeUpdateDynamicBoundingBoxes(socket);
      } catch (err) {
        // Keep timer alive if one cycle fails.
        console.error('Periodic flush/update cycle failed.', err);
      }
    }, DEFAULT_FLUSH_INTERVAL_MS);
  };

  socket.onmessage = async (event) => processMessage(event.data);

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
    scheduleReconnect();
  };
};

const scheduleReconnect = () => {
  reconnectAttempts += 1;
  const backoff = Math.min(30000, 1000 * 2 ** reconnectAttempts);
  setTimeout(startWebSocket, backoff);
};

const main = async () => {
  console.log('Starting AIS ingest worker');
  await mongoose.connect(MONGODB_URI);
  console.log('Connected to MongoDB');
  // Initialize dynamic bbox state after DB connection so GPS queries can run.
  await initializeBoundingBoxesFromBoatGps();
  startWebSocket();
};

process.on('SIGINT', async () => {
  console.log('Shutting down...');
  if (flushIntervalId) {
    clearInterval(flushIntervalId);
  }
  await flushSnapshot();
  await mongoose.disconnect();
  process.exit(0);
});

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
