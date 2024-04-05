export const decimal2JSON = (v, i, prev) => {
  if (v !== null && typeof v === 'object') {
    if (v.constructor.name === 'Decimal128') prev[i] = parseFloat(v.toString());
    else
      Object.entries(v).forEach(([key, value]) =>
        decimal2JSON(value, key, prev ? prev[i] : v),
      );
  }
};
