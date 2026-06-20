/**
 * Pure geometry helpers for the AIS dynamic bounding box.
 *
 * Extracted from aisstream-ingest.js so they can be unit-tested in isolation.
 * No I/O, no module-level state — every function is deterministic in its args.
 */

const EARTH_RADIUS_KM = 6371;

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
    Math.cos(toRadians(lat1)) *
      Math.cos(toRadians(lat2)) *
      Math.sin(deltaLon / 2) ** 2;
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
    return [
      [
        [south, -180],
        [north, 180],
      ],
    ];
  }

  const rawWest = longitude - longitudeDelta;
  const rawEast = longitude + longitudeDelta;

  if (rawEast > 180 || rawWest < -180) {
    return [
      [
        [south, normalizeLongitude(rawWest)],
        [north, 180],
      ],
      [
        [south, -180],
        [north, normalizeLongitude(rawEast)],
      ],
    ];
  }

  return [
    [
      [south, rawWest],
      [north, rawEast],
    ],
  ];
};

module.exports = {
  EARTH_RADIUS_KM,
  clampLatitude,
  normalizeLongitude,
  toRadians,
  haversineKm,
  buildBoundingBoxesAroundCenter,
};
