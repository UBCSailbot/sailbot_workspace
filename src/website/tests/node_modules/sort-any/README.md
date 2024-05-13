# sort-any

[![MIT License](https://img.shields.io/badge/license-mit-green.svg?style=flat-square)](https://opensource.org/licenses/MIT)
[![Build Status](https://travis-ci.org/oprogramador/sort-any.svg?branch=master)](https://travis-ci.org/oprogramador/sort-any
)

[![NPM status](https://nodei.co/npm/sort-any.png?downloads=true&stars=true)](https://npmjs.org/package/sort-any)

JS library which always sorts arrays in a predictable way. Moreover in contrary to `Array.prototype.sort`, it does not modify the argument.

`Array.prototype.sort`:
```js
[[[11]],[[2]],2,{foo:{foo:2}},1,4,2,{foo:{foo:1}},32,[3,4],[1,2],{foo:1},{foo:0},{foo:{}}].sort()
/*
It gives:
[ 1,
  [ 1, 2 ],
  [ [ 11 ] ],
  2,
  2,
  [ [ 2 ] ],
  [ 3, 4 ],
  32,
  4,
  { foo: { foo: 2 } },
  { foo: {} },
  { foo: 1 },
  { foo: 0 },
  { foo: { foo: 1 } } ]
*/
```

The same array in a different order:
```js
[[[11]],[[2]],2,1,4,2,{foo:{foo:1}},32,[3,4],[1,2],{foo:0},{foo:1},{foo:{}},{foo:{foo:2}}].sort()
/*
It gives:
[ 1,
  [ 1, 2 ],
  [ [ 11 ] ],
  2,
  2,
  [ [ 2 ] ],
  [ 3, 4 ],
  32,
  4,
  { foo: { foo: 1 } },
  { foo: 0 },
  { foo: 1 },
  { foo: {} },
  { foo: { foo: 2 } } ]
*/
```
So the results of `Array.prototype.sort` are strange (eg. numbers are sorted in the alphabetical order) and moreover if we change the array order (eg. for object items), the result has order changed as well.

So I have implemented this library to work like that:
```js
const sort = require('sort-any');
sort([[[11]],[[2]],2,{foo:{foo:2}},1,4,2,{foo:{foo:1}},32,[3,4],[1,2],{foo:1},{foo:0},{foo:{}}])
/*
It returns:
[ 1,
  2,
  2,
  4,
  32,
  [ [ 2 ] ],
  [ [ 11 ] ],
  [ 1, 2 ],
  [ 3, 4 ],
  { foo: 0 },
  { foo: 1 },
  { foo: {} },
  { foo: { foo: 1 } },
  { foo: { foo: 2 } } ]
*/
```

And when we change the order, the result remains the same.
```js
const sort = require('sort-any');
sort([[[11]],[[2]],2,1,4,2,{foo:{foo:1}},32,[3,4],[1,2],{foo:0},{foo:1},{foo:{}},{foo:{foo:2}}])
/*
It returns:
[ 1,
  2,
  2,
  4,
  32,
  [ [ 2 ] ],
  [ [ 11 ] ],
  [ 1, 2 ],
  [ 3, 4 ],
  { foo: 0 },
  { foo: 1 },
  { foo: {} },
  { foo: { foo: 1 } },
  { foo: { foo: 2 } } ]
*/
```

Rules for sorting:
- the most important is the type (from the smallest to the largest):
  - undefined
  - null
  - boolean
  - NaN
  - number (all the numbers except of NaN)
  - string
  - symbol
  - date
  - array
  - object (all the objects except of arrays, dates and null)
- `false` is less than `true`
- numbers are sorted with the standard numeric order
- `-Infinity` is less than any other number
- `Infinity` is more than any other number
- strings are sorted in the alphabetical order
- symbols are sorted in the alphabetical order according to their description
- dates are sorted in the chronological order
- rules of arrays sorting:
  - the most important is the length (always a shorter array is less than a longer array)
  - if the length is the same, we sort (recursively using this algorithm) both arrays and we compare the smallest item from both arrays
  - if the smallest item is the same, we compare the second smallest item, and so on
  - if the arrays include the same values, we compare the elements at 0 index
  - if the elements at 0 index are the same, we compare the elements at 1 index, 2 index, and so on
  - if all the elements are equal, the arrays are equal
- rules of objects sorting:
  - the most important is the number of the keys (always an object with less keys is less than an object with more keys)
  - if the number of keys is the same, we sort (recursively using this algorithm) the keys (which can be either strings or symbols) and we compare the smallest keys from both objects
  - if the smallest key is the same, we compare the second smallest key, and so on
  - if all the keys are the same, we compare the values at the smallest key
  - if the values at the smallest key are the same, we compare sequentially next values taking each time a bigger key unless the values differ
  - if all the values are the same, we compare the keys at 0 index (`Object.keys(object)[0]`)
  - if the keys at 0 index are the same, we compare the keys at 1 index, 2 index, and so on
  - if all the keys are the same, the objects are equal
