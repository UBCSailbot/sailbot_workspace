import mongoose from 'mongoose';

import { decimal2JSON } from './helper/parser';

export interface GPS extends mongoose.Document {
  latitude: mongoose.Types.Decimal128;
  longitude: mongoose.Types.Decimal128;
  speed: mongoose.Types.Decimal128;
  heading: mongoose.Types.Decimal128;
  timestamp: string;
}

const GPSSchema = new mongoose.Schema<GPS>({
  latitude: {
    type: mongoose.Types.Decimal128,
    required: [true, "Missing 'latitude' field in GPS interface"],
  },
  longitude: {
    type: mongoose.Types.Decimal128,
    required: [true, "Missing 'longitude' field in GPS interface"],
  },
  speed: {
    type: mongoose.Types.Decimal128,
    required: [true, "Missing 'speed' field in GPS interface"],
  },
  heading: {
    type: mongoose.Types.Decimal128,
    required: [true, "Missing 'heading' field in GPS interface"],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

GPSSchema.set('toJSON', {
  transform: (doc, ret) => {
    // @ts-ignore: Expected 3 arguments, but got 1
    decimal2JSON(ret);
    return ret;
  },
});

export default mongoose.models.GPS || mongoose.model<GPS>('GPS', GPSSchema);
