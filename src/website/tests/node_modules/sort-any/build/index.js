'use strict';

const _ = require('lodash');

/* eslint-disable no-use-before-define */
/* eslint-disable sort-keys */

const types = {
  undefined: Symbol('undefined'),
  null: Symbol('null'),
  boolean: Symbol('boolean'),
  NaN: Symbol('NaN'),
  number: Symbol('number'),
  string: Symbol('string'),
  symbol: Symbol('symbol'),
  date: Symbol('date'),
  array: Symbol('array'),
  object: Symbol('object')
};

const typesValues = _.values(types);
const orderedTypes = _.zipObject(typesValues, Object.keys(typesValues).map(key => Number(key)));

const comparators = {
  [types.array]: compareArray,
  [types.number]: standardCompare,
  [types.object]: compareObject,
  [types.string]: standardCompare,
  [types.symbol]: (a, b) => standardCompare(a.toString().slice(0, -1), b.toString().slice(0, -1))
};

function getOrderByType(type) {
  return orderedTypes[type];
}

function getTypeByValue(value) {
  if (typeof value === 'undefined') {
    return types.undefined;
  }
  if (value === null) {
    return types.null;
  }
  if (typeof value === 'boolean') {
    return types.boolean;
  }
  if (typeof value === 'number' && Number.isNaN(value)) {
    return types.NaN;
  }
  if (typeof value === 'number') {
    return types.number;
  }
  if (typeof value === 'string') {
    return types.string;
  }
  if (typeof value === 'symbol') {
    return types.symbol;
  }
  if (value instanceof Date) {
    return types.date;
  }
  if (Array.isArray(value)) {
    return types.array;
  }

  return types.object;
}

function standardCompare(a, b) {
  if (a < b) {
    return -1;
  }
  if (a > b) {
    return 1;
  }

  return 0;
}

function compareArray(first, second) {
  if (first.length < second.length) {
    return -1;
  }
  if (second.length < first.length) {
    return 1;
  }
  const sortedFirst = sortAny(first);
  const sortedSecond = sortAny(second);

  for (let i = 0; i < first.length; i++) {
    const compareResult = compareSimple(sortedFirst[i], sortedSecond[i]);
    if (compareResult) {
      return compareResult;
    }
  }

  for (let i = 0; i < first.length; i++) {
    const compareResult = compareSimple(first[i], second[i]);
    if (compareResult) {
      return compareResult;
    }
  }

  return 0;
}

function compareObject(first, second) {
  const firstKeys = Object.keys(first);
  const secondKeys = Object.keys(second);
  if (firstKeys.length < secondKeys.length) {
    return -1;
  }
  if (secondKeys.length < firstKeys.length) {
    return 1;
  }
  const sortedFirstKeys = sortAny(firstKeys);
  const sortedSecondKeys = sortAny(secondKeys);

  for (let i = 0; i < firstKeys.length; i++) {
    const compareResult = compareSimple(sortedFirstKeys[i], sortedSecondKeys[i]);
    if (compareResult) {
      return compareResult;
    }
  }

  for (let i = 0; i < firstKeys.length; i++) {
    const key = sortedFirstKeys[i];
    const compareResult = compareSimple(first[key], second[key]);
    if (compareResult) {
      return compareResult;
    }
  }

  for (let i = 0; i < firstKeys.length; i++) {
    const compareResult = compareSimple(firstKeys[i], secondKeys[i]);
    if (compareResult) {
      return compareResult;
    }
  }

  return 0;
}

function compareSimple(first, second) {
  const firstType = getTypeByValue(first);
  const secondType = getTypeByValue(second);
  const firstOrder = getOrderByType(firstType);
  const secondOrder = getOrderByType(secondType);
  const differenceByType = firstOrder - secondOrder;
  if (differenceByType) {
    return differenceByType;
  }
  const comparator = comparators[firstType] || standardCompare;

  return comparator(first, second);
}

function compare(a, b) {
  return compareSimple(a, b);
}

function sortAny(array) {
  const undefinedsArray = array.filter(x => typeof x === 'undefined');
  const notUndefinedsArray = array.filter(x => typeof x !== 'undefined');

  return [...undefinedsArray, ...[...notUndefinedsArray].sort(compare)];
}

module.exports = sortAny;