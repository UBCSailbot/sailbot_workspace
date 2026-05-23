export const decimal2JSON = (v, i, prev) => {
  if (v !== null && typeof v === 'object') {
    if (v.constructor.name === 'Decimal128') prev[i] = parseFloat(v.toString());
    else
      Object.entries(v).forEach(([key, value]) =>
        decimal2JSON(value, key, prev ? prev[i] : v),
      );
  }
};

/**
 * Converts a compact timestamp string to Unix seconds (UTC).
 * @param {string} ts - Timestamp in "YY-MM-DD HH:MM:SS" format (year offset from 2000).
 * @returns {number} Unix timestamp in seconds.
 * @returns {number} 0 if `ts` does not match the expected format.
 */
export const normalizeTimestamp = (ts) => {
  const match = ts.match(/^(\d{2})-(\d{2})-(\d{2}) (\d{2}):(\d{2}):(\d{2})$/);
  if (!match) return 0;
  const [, yy, mo, dd, hh, mm, ss] = match;
  return Date.UTC(2000 + +yy, +mo - 1, +dd, +hh, +mm, +ss) / 1000;
};
