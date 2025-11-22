/**
 * AIS Stream ingest worker.
 * - Connects to aisstream.io via WebSocket using your API key.
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
const POSITION_MESSAGE_TYPES = [
  'PositionReport',
  'StandardClassBPositionReport',
  'ExtendedClassBPositionReport',
  'ShipStaticData',
];

const parseBoundingBoxes = (raw) => {
  if (!raw) {
    return [[[-90, -180], [90, 180]]];
  }
  try {
    const parsed = JSON.parse(raw);
    if (Array.isArray(parsed) && parsed.length > 0) {
      return parsed;
    }
  } catch (err) {
    console.warn('Could not parse AISSTREAM_BOUNDING_BOXES, using world coverage.', err);
  }
  return [[[-90, -180], [90, 180]]];
};

const parseList = (raw) =>
  raw
    ? raw
        .split(',')
        .map((item) => item.trim())
        .filter(Boolean)
    : [];

const boundingBoxes = parseBoundingBoxes(AISSTREAM_BOUNDING_BOXES);
const mmsiFilter = parseList(AISSTREAM_FILTER_SHIP_MMSI);
const messageTypeFilter =
  parseList(AISSTREAM_FILTER_MESSAGE_TYPES).length > 0
    ? parseList(AISSTREAM_FILTER_MESSAGE_TYPES)
    : POSITION_MESSAGE_TYPES;

const subscriptionPayload = {
  APIKey: AISSTREAM_API_KEY,
  BoundingBoxes: boundingBoxes,
  FilterMessageTypes: messageTypeFilter,
};
if (mmsiFilter.length > 0) {
  subscriptionPayload.FiltersShipMMSI = mmsiFilter;
}

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

const processMessage = (rawData) => {
  let parsed;
  try {
    const payload =
      typeof rawData === 'string' ? rawData : rawData.toString('utf8');
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
  console.log('Connecting to aisstream.io ...');
  const socket = new WebSocket('wss://stream.aisstream.io/v0/stream');

  socket.onopen = () => {
    reconnectAttempts = 0;
    try {
      socket.send(JSON.stringify(subscriptionPayload));
      console.log('Subscription sent to aisstream.io');
    } catch (err) {
      console.error('Failed to send subscription message', err);
    }

    if (flushIntervalId) {
      clearInterval(flushIntervalId);
    }
    flushIntervalId = setInterval(flushSnapshot, DEFAULT_FLUSH_INTERVAL_MS);
  };

  socket.onmessage = (event) => processMessage(event.data);

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
