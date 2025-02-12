import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import GenericSensors from '@/models/GenericSensors';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const genericSensors = await GenericSensors.find({}).select({
          'genericSensors._id': 0,
          _id: 0,
          __v: 0,
        });

        res.status(200).json({ success: true, data: genericSensors });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { genericSensors, timestamp } = req.body;

        if (!Array.isArray(genericSensors) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid GenericSensors data format" });
        }

        const processedSensors = genericSensors.map(sensor => ({
          id: sensor.id,
          data: typeof sensor.data === "bigint" ? sensor.data.toString() : sensor.data
        }));

        const newGenericSensors = new GenericSensors({ genericSensors: processedSensors, timestamp });
        await newGenericSensors.save();

        res.status(201).json({ success: true, message: "GenericSensors data stored", data: newGenericSensors });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'DELETE':
      try {
        await GenericSensors.deleteMany({});
        res.status(200).json({ success: true, message: "All GenericSensors data deleted" });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
