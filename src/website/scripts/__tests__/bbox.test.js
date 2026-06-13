/**
 * Unit tests for the bbox geometry helpers.
 *
 * Run with:
 *   node --test src/website/scripts/__tests__/bbox.test.js
 *
 * Uses node:test + node:assert (built-in, no devDependencies).
 */

const test = require('node:test');
const assert = require('node:assert/strict');

const {
  EARTH_RADIUS_KM,
  clampLatitude,
  normalizeLongitude,
  haversineKm,
  buildBoundingBoxesAroundCenter,
} = require('../lib/bbox');

const KM_PER_DEGREE_LAT = 111;
// Allow ~1% slack on great-circle math (the helper uses a flat 111 km/deg
// approximation; geodetic reality varies ~0.5%).
const closeTo = (actual, expected, tolerance) =>
  Math.abs(actual - expected) <= tolerance;

test('clampLatitude keeps in-range values untouched', () => {
  assert.equal(clampLatitude(0), 0);
  assert.equal(clampLatitude(45), 45);
  assert.equal(clampLatitude(-89.9), -89.9);
});

test('clampLatitude clamps to ±90', () => {
  assert.equal(clampLatitude(120), 90);
  assert.equal(clampLatitude(-120), -90);
  assert.equal(clampLatitude(90), 90);
  assert.equal(clampLatitude(-90), -90);
});

test('normalizeLongitude is identity for in-range values', () => {
  assert.equal(normalizeLongitude(0), 0);
  assert.equal(normalizeLongitude(179), 179);
  assert.equal(normalizeLongitude(-179), -179);
  assert.equal(normalizeLongitude(180), 180);
});

test('normalizeLongitude wraps single overflow', () => {
  assert.equal(normalizeLongitude(181), -179);
  assert.equal(normalizeLongitude(-181), 179);
  assert.equal(normalizeLongitude(190), -170);
});

test('normalizeLongitude wraps multiple revolutions', () => {
  assert.equal(normalizeLongitude(360 + 10), 10);
  assert.equal(normalizeLongitude(-360 - 10), -10);
  assert.equal(normalizeLongitude(720 + 5), 5);
});

test('haversineKm is zero for identical points', () => {
  assert.equal(haversineKm(0, 0, 0, 0), 0);
  assert.equal(haversineKm(49.28, -123.12, 49.28, -123.12), 0);
});

test('haversineKm matches one degree of latitude (~111 km)', () => {
  const km = haversineKm(0, 0, 1, 0);
  assert.ok(closeTo(km, KM_PER_DEGREE_LAT, 1), `expected ~111 km, got ${km}`);
});

test('haversineKm matches one degree of longitude at the equator', () => {
  const km = haversineKm(0, 0, 0, 1);
  assert.ok(closeTo(km, KM_PER_DEGREE_LAT, 1), `expected ~111 km, got ${km}`);
});

test('haversineKm shrinks longitude distance at high latitude', () => {
  // At 60°N one degree of longitude is about 111 * cos(60°) = ~55.5 km.
  const km = haversineKm(60, 0, 60, 1);
  assert.ok(closeTo(km, 55.5, 1), `expected ~55.5 km, got ${km}`);
});

test('haversineKm caps at half-circumference for antipodal points', () => {
  // Antipodal points are about π * R km apart (~20015 km).
  const km = haversineKm(0, 0, 0, 180);
  const expected = Math.PI * EARTH_RADIUS_KM;
  assert.ok(closeTo(km, expected, 1), `expected ~${expected} km, got ${km}`);
});

test('haversineKm is symmetric in argument order', () => {
  const ab = haversineKm(10, 20, 30, 40);
  const ba = haversineKm(30, 40, 10, 20);
  assert.ok(Math.abs(ab - ba) < 1e-9);
});

test('buildBoundingBoxesAroundCenter returns a single box for an interior point', () => {
  const boxes = buildBoundingBoxesAroundCenter(0, 0, 30);
  assert.equal(boxes.length, 1);
  const [[south, west], [north, east]] = boxes[0];
  // 30 km is about 0.27 degrees at the equator.
  assert.ok(closeTo(north - south, 0.54, 0.05));
  assert.ok(closeTo(east - west, 0.54, 0.05));
  // Box must be centered on the origin.
  assert.ok(closeTo((north + south) / 2, 0, 1e-9));
  assert.ok(closeTo((east + west) / 2, 0, 1e-9));
  // AISstream requires west < east within each box.
  assert.ok(west < east);
});

test('buildBoundingBoxesAroundCenter scales longitude span with latitude', () => {
  const equator = buildBoundingBoxesAroundCenter(0, 0, 30)[0];
  const sixtyNorth = buildBoundingBoxesAroundCenter(60, 0, 30)[0];
  const equatorLonSpan = equator[1][1] - equator[0][1];
  const sixtyLonSpan = sixtyNorth[1][1] - sixtyNorth[0][1];
  // cos(60°) = 0.5, so the box at 60°N should be ~2× wider in degrees.
  assert.ok(
    closeTo(sixtyLonSpan, equatorLonSpan * 2, 0.05),
    `expected sixtyLonSpan ≈ 2× equatorLonSpan, got ${sixtyLonSpan} vs ${equatorLonSpan}`,
  );
});

test('buildBoundingBoxesAroundCenter clamps latitude at the poles', () => {
  // Centering near the north pole should clamp the northern edge to 90°.
  const boxes = buildBoundingBoxesAroundCenter(89.9, 0, 30);
  assert.equal(boxes.length, 1);
  const [, [north]] = boxes[0];
  assert.equal(north, 90);
});

test('buildBoundingBoxesAroundCenter splits across the antimeridian (east side)', () => {
  // Center just west of the antimeridian: east edge spills past +180.
  const boxes = buildBoundingBoxesAroundCenter(0, 179.9, 30);
  assert.equal(boxes.length, 2);
  // First box ends at +180, second starts at -180. Each must satisfy west < east.
  for (const [[, west], [, east]] of boxes) {
    assert.ok(west < east, `box violates west<east: west=${west} east=${east}`);
  }
  const easts = boxes.map((b) => b[1][1]);
  const wests = boxes.map((b) => b[0][1]);
  assert.ok(easts.includes(180));
  assert.ok(wests.includes(-180));
});

test('buildBoundingBoxesAroundCenter splits across the antimeridian (west side)', () => {
  const boxes = buildBoundingBoxesAroundCenter(0, -179.9, 30);
  assert.equal(boxes.length, 2);
  for (const [[, west], [, east]] of boxes) {
    assert.ok(west < east);
  }
});

test('buildBoundingBoxesAroundCenter returns full longitude range when window >= 360°', () => {
  // A radius huge enough that longitudeDelta >= 180.
  const boxes = buildBoundingBoxesAroundCenter(0, 0, 30000);
  assert.equal(boxes.length, 1);
  const [[south, west], [north, east]] = boxes[0];
  assert.equal(west, -180);
  assert.equal(east, 180);
  // Latitude also gets clamped by the 30000 km radius.
  assert.equal(south, -90);
  assert.equal(north, 90);
});

test('buildBoundingBoxesAroundCenter survives the polar cos(lat) floor', () => {
  // At lat=90, cos(lat)=0; the 0.1 floor keeps longitudeDelta finite. The box
  // should still be returnable and have west < east.
  const boxes = buildBoundingBoxesAroundCenter(90, 0, 30);
  assert.ok(boxes.length >= 1);
  for (const box of boxes) {
    const [[, west], [, east]] = box;
    assert.ok(west < east, `box violates west<east at pole: ${west} ${east}`);
  }
});
