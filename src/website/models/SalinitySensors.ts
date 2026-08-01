import mongoose from 'mongoose';

interface SalinitySensor extends mongoose.Document {
  salinity: number;
}

export interface SalinitySensors extends mongoose.Document {
  salinitySensors: SalinitySensor[];
  timestamp: string;
}

const SalinitySensorsSchema = new mongoose.Schema<SalinitySensors>({
  salinitySensors: {
    type: [
      {
        salinity: Number,
      },
    ],
    required: [true, 'Missing array of objects in SalinitySensors interface'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

export default (mongoose.models['SalinitySensors'] as
  | mongoose.Model<SalinitySensors>
  | undefined) ??
  mongoose.model<SalinitySensors>(
    'SalinitySensors',
    SalinitySensorsSchema,
    'salinity_sensors',
  );
