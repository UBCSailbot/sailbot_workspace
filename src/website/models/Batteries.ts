import mongoose from 'mongoose';

import { decimal2JSON } from './helper/parser';

interface Battery extends mongoose.Document {
  voltage: mongoose.Types.Decimal128;
  current: mongoose.Types.Decimal128;
}

export interface Batteries extends mongoose.Document {
  batteries: Battery[];
  timestamp: string;
}

const BatteriesSchema = new mongoose.Schema<Batteries>({
  batteries: {
    type: [
      {
        voltage: mongoose.Types.Decimal128,
        current: mongoose.Types.Decimal128,
      },
    ],
    required: [true, 'Missing array of objects in Batteries interface'],
    validate: [
      validateArrayLimit,
      'The array length of {PATH} should equal to 2.',
    ],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

function validateArrayLimit(val: any) {
  return val.length == 2;
}

BatteriesSchema.set('toJSON', {
  transform: (doc, ret) => {
    // @ts-ignore: Expected 3 arguments, but got 1
    decimal2JSON(ret);
    return ret;
  },
});

export default mongoose.models.Batteries ||
  mongoose.model<Batteries>('Batteries', BatteriesSchema);
