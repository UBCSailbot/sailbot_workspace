import mongoose from 'mongoose';

interface TempSensor extends mongoose.Document {
  temperature: number;
}

export interface TempSensors extends mongoose.Document {
  tempSensors: TempSensor[];
  timestamp: string;
}

const TempSensorsSchema = new mongoose.Schema<TempSensors>({
  tempSensors: {
    type: [
      {
        temperature: Number,
      },
    ],
    required: [true, 'Missing array of objects in TempSensors interface'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

export default (mongoose.models['TempSensors'] as
  | mongoose.Model<TempSensors>
  | undefined) ??
  mongoose.model<TempSensors>('TempSensors', TempSensorsSchema, 'temp_sensors');
