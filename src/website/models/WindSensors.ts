import mongoose from 'mongoose';

import { decimal2JSON } from './helper/parser';

interface WindSensor extends mongoose.Document {
  speed: mongoose.Types.Decimal128;
  direction: number;
}

export interface WindSensors extends mongoose.Document {
  windSensors: WindSensor[];
  timestamp: string;
}

const WindSensorsSchema = new mongoose.Schema<WindSensors>({
  windSensors: {
    type: [
      {
        speed: mongoose.Types.Decimal128,
        direction: Number,
      },
    ],
    required: [true, 'Missing array of objects in WindSensors interface'],
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

WindSensorsSchema.set('toJSON', {
  transform: (doc, ret) => {
    // @ts-ignore: Expected 3 arguments, but got 1
    decimal2JSON(ret);
    return ret;
  },
});

export default mongoose.models.WindSensors ||
  mongoose.model<WindSensors>('WindSensors', WindSensorsSchema);
