import mongoose from 'mongoose';

interface PhSensor extends mongoose.Document {
  ph: number;
}

export interface PhSensors extends mongoose.Document {
  phSensors: PhSensor[];
  timestamp: string;
}

const PhSensorsSchema = new mongoose.Schema<PhSensors>({
  phSensors: {
    type: [
      {
        ph: Number,
      },
    ],
    required: [true, 'Missing array of objects in PhSensors interface'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

export default (mongoose.models['PhSensors'] as
  | mongoose.Model<PhSensors>
  | undefined) ??
  mongoose.model<PhSensors>('PhSensors', PhSensorsSchema, 'ph_sensors');
