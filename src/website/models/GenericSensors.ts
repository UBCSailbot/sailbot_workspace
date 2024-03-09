import mongoose from 'mongoose';

interface GenericSensor extends mongoose.Document {
  id: number;
  data: bigint;
}

export interface GenericSensors extends mongoose.Document {
  genericSensors: GenericSensor[];
  timestamp: string;
}

const GenericSensorsSchema = new mongoose.Schema<GenericSensors>({
  genericSensors: {
    type: [
      {
        id: Number,
        data: BigInt,
      },
    ],
    required: [true, 'Missing array of objects in GenericSensors interface'],
  },
  timestamp: {
    type: String,
    default: () => new Date().toISOString(),
  },
});

/* Convert BigInt to a string */
(BigInt.prototype as any).toJSON = function () {
  return this.toString();
};
(BigInt.prototype as any).toBSON = function () {
  return this.toString();
};

export default mongoose.models.GenericSensors ||
  mongoose.model<GenericSensors>('GenericSensors', GenericSensorsSchema);
