import mongoose from 'mongoose';

import { decimal2JSON } from './helper/parser';

interface WayPoint extends mongoose.Document {
  latitude: mongoose.Types.Decimal128;
  longitude: mongoose.Types.Decimal128;
}

export interface LocalPath extends mongoose.Document {
  waypoints: WayPoint[];
  timestamp: string;
}

const LocalPathSchema = new mongoose.Schema<LocalPath>({
  waypoints: {
    type: [
      {
        latitude: mongoose.Types.Decimal128,
        longitude: mongoose.Types.Decimal128,
      },
    ],
    required: [true, 'Missing array of objects in LocalPath interface'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

LocalPathSchema.set('toJSON', {
  transform: (doc, ret) => {
    // @ts-ignore: Expected 3 arguments, but got 1
    decimal2JSON(ret);
    return ret;
  },
});

export default mongoose.models.LocalPath ||
  mongoose.model<LocalPath>('LocalPath', LocalPathSchema);
