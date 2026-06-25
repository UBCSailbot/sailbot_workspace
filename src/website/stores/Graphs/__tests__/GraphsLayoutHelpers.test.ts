/**
 * Unit tests for the Graphs layout helpers.
 *
 * Run with:
 *   npm run test:graphs
 *
 * Uses node:test + node:assert. Node 25 strips TypeScript by default
 * (process.features.typescript === 'strip'), so no transpile step is needed.
 */

import test from 'node:test';
import assert from 'node:assert/strict';

import {
  findLayoutIndex,
  moveGraph,
  moveGraphToIndex,
  extractGraph,
  splitGraph,
} from '../GraphsLayoutHelpers.ts';
import type { Layout } from '../GraphsTypes.ts';
import { isSplitGroup, getAllGraphIds } from '../GraphsTypes.ts';

const DEFAULT: Layout = [
  'GPS',
  'BatteriesVoltage',
  'BatteriesCurrent',
  'WindSensors',
];

test('findLayoutIndex finds standalone items', () => {
  assert.equal(findLayoutIndex(DEFAULT, 'GPS'), 0);
  assert.equal(findLayoutIndex(DEFAULT, 'WindSensors'), 3);
});

test('findLayoutIndex finds items inside a split group', () => {
  const layout: Layout = ['GPS', ['BatteriesVoltage', 'BatteriesCurrent'], 'WindSensors'];
  assert.equal(findLayoutIndex(layout, 'BatteriesVoltage'), 1);
  assert.equal(findLayoutIndex(layout, 'BatteriesCurrent'), 1);
});

test('findLayoutIndex returns -1 for missing graphs', () => {
  // @ts-expect-error — runtime guard, intentionally passing an unknown id
  assert.equal(findLayoutIndex(DEFAULT, 'NotAGraph'), -1);
});

test('isSplitGroup correctly discriminates the union', () => {
  assert.equal(isSplitGroup('GPS'), false);
  assert.equal(isSplitGroup(['GPS', 'BatteriesVoltage']), true);
});

test('getAllGraphIds flattens nested split groups', () => {
  const layout: Layout = ['GPS', ['BatteriesVoltage', 'BatteriesCurrent'], 'WindSensors'];
  assert.deepEqual(getAllGraphIds(layout), [
    'GPS',
    'BatteriesVoltage',
    'BatteriesCurrent',
    'WindSensors',
  ]);
});

test('moveGraph inserts the source just before the target', () => {
  // Drag-to-reorder semantics: source slides to the target's position.
  assert.deepEqual(moveGraph(DEFAULT, 'GPS', 'WindSensors'), [
    'BatteriesVoltage',
    'BatteriesCurrent',
    'GPS',
    'WindSensors',
  ]);
});

test('moveGraph is a no-op when sourceId === targetId', () => {
  assert.equal(moveGraph(DEFAULT, 'GPS', 'GPS'), DEFAULT);
});

test('moveGraph is a no-op when both ids share a layout item', () => {
  const layout: Layout = [['GPS', 'BatteriesVoltage'], 'BatteriesCurrent', 'WindSensors'];
  assert.equal(moveGraph(layout, 'GPS', 'BatteriesVoltage'), layout);
});

test('moveGraph treats a split group as an atomic unit', () => {
  const layout: Layout = [['GPS', 'BatteriesVoltage'], 'BatteriesCurrent', 'WindSensors'];
  // The whole [GPS, BV] group slides to just before WindSensors.
  assert.deepEqual(moveGraph(layout, 'GPS', 'WindSensors'), [
    'BatteriesCurrent',
    ['GPS', 'BatteriesVoltage'],
    'WindSensors',
  ]);
});

test('moveGraphToIndex inserts at the end when targetIndex === layout.length', () => {
  assert.deepEqual(moveGraphToIndex(DEFAULT, 'GPS', 4), [
    'BatteriesVoltage',
    'BatteriesCurrent',
    'WindSensors',
    'GPS',
  ]);
});

test('moveGraphToIndex no-ops when target gap is adjacent to source', () => {
  // GPS sits at index 0. Gaps 0 and 1 are adjacent → no-op.
  assert.equal(moveGraphToIndex(DEFAULT, 'GPS', 0), DEFAULT);
  assert.equal(moveGraphToIndex(DEFAULT, 'GPS', 1), DEFAULT);
});

test('moveGraphToIndex shifts items correctly', () => {
  assert.deepEqual(moveGraphToIndex(DEFAULT, 'WindSensors', 0), [
    'WindSensors',
    'GPS',
    'BatteriesVoltage',
    'BatteriesCurrent',
  ]);
});

test('extractGraph pulls a graph out of its split group as a standalone item', () => {
  const layout: Layout = ['GPS', ['BatteriesVoltage', 'BatteriesCurrent'], 'WindSensors'];
  // Extract BatteriesVoltage and drop it at gap 0 (start of layout).
  assert.deepEqual(extractGraph(layout, 'BatteriesVoltage', 0), [
    'BatteriesVoltage',
    'GPS',
    'BatteriesCurrent',
    'WindSensors',
  ]);
});

test('extractGraph collapses the remaining sibling to a standalone item', () => {
  // After extraction the partner is left alone in the split group — removeGraph
  // collapses 1-member groups so the layout never holds a degenerate group.
  const layout: Layout = [['GPS', 'BatteriesVoltage'], 'BatteriesCurrent', 'WindSensors'];
  // Layout has 3 items, so gap 3 is "insert at end". After removeGraph collapses
  // [GPS, BV] → BV, the intermediate layout is [BV, BC, WS]; GPS lands at index 3.
  const result = extractGraph(layout, 'GPS', 3);
  assert.deepEqual(result, [
    'BatteriesVoltage',
    'BatteriesCurrent',
    'WindSensors',
    'GPS',
  ]);
  // Confirm there are no nested groups anywhere.
  for (const item of result) {
    assert.equal(isSplitGroup(item), false);
  }
});

test('extractGraph delegates to moveGraphToIndex for standalone graphs', () => {
  assert.deepEqual(extractGraph(DEFAULT, 'WindSensors', 0), [
    'WindSensors',
    'GPS',
    'BatteriesVoltage',
    'BatteriesCurrent',
  ]);
});

test('splitGraph creates a 2-member row at the target position', () => {
  assert.deepEqual(splitGraph(DEFAULT, 'GPS', 'BatteriesCurrent', 'right'), [
    'BatteriesVoltage',
    ['BatteriesCurrent', 'GPS'],
    'WindSensors',
  ]);
});

test('splitGraph honors the side argument', () => {
  assert.deepEqual(splitGraph(DEFAULT, 'GPS', 'BatteriesCurrent', 'left'), [
    'BatteriesVoltage',
    ['GPS', 'BatteriesCurrent'],
    'WindSensors',
  ]);
});

test('splitGraph defaults side to "right"', () => {
  assert.deepEqual(splitGraph(DEFAULT, 'GPS', 'BatteriesCurrent'), [
    'BatteriesVoltage',
    ['BatteriesCurrent', 'GPS'],
    'WindSensors',
  ]);
});

test('splitGraph is a no-op when target is already in a 2-member group', () => {
  // Two-per-row cap (minjunminji's request, enforced here).
  const layout: Layout = ['GPS', ['BatteriesVoltage', 'BatteriesCurrent'], 'WindSensors'];
  assert.deepEqual(splitGraph(layout, 'WindSensors', 'BatteriesVoltage'), layout);
});

test('splitGraph is a no-op when source === target', () => {
  assert.equal(splitGraph(DEFAULT, 'GPS', 'GPS'), DEFAULT);
});

test('splitGraph is a no-op when source and target already share a layout item', () => {
  const layout: Layout = [['GPS', 'BatteriesVoltage'], 'BatteriesCurrent', 'WindSensors'];
  // Already grouped — splitting them again is meaningless.
  assert.deepEqual(splitGraph(layout, 'GPS', 'BatteriesVoltage'), layout);
});

test('the layout invariant holds across a sequence of operations', () => {
  // Build a split, then extract one side, then re-split.
  let layout: Layout = [...DEFAULT];
  layout = splitGraph(layout, 'GPS', 'BatteriesCurrent', 'left');
  layout = extractGraph(layout, 'GPS', 0);
  layout = splitGraph(layout, 'WindSensors', 'BatteriesVoltage', 'right');

  // No nested groups, no duplicates, all 4 graphs present.
  assert.deepEqual(getAllGraphIds(layout).sort(), [
    'BatteriesCurrent',
    'BatteriesVoltage',
    'GPS',
    'WindSensors',
  ]);
  for (const item of layout) {
    if (isSplitGroup(item)) {
      assert.ok(item.length === 2, `unexpected group size: ${item.length}`);
    }
  }
});
