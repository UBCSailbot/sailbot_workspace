import mongoose from 'mongoose';

import { decimal2JSON } from './helper/parser';

export interface DesiredHeading extends mongoose.Document {
  heading: mongoose.Types.Decimal128;
}

const DesiredHeadingSchema = new mongoose.Schema<DesiredHeading>({
  heading: {
    type: mongoose.Types.Decimal128,
    required: [true, "Missing 'heading' field in DesiredHeading interface"],
  },
});

DesiredHeadingSchema.set('toJSON', {
  transform: (doc, ret) => {
    // @ts-ignore: Expected 3 arguments, but got 1
    decimal2JSON(ret);
    return ret;
  },
});

export default mongoose.models.DesiredHeading ||
  mongoose.model<DesiredHeading>('DesiredHeading', DesiredHeadingSchema);
