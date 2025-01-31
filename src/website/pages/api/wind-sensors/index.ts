import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import WindSensors from '@/models/WindSensors';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const windSensors = await WindSensors.find({}).select({
          'windSensors._id': 0,
          _id: 0,
          __v: 0,
        });
        res.status(200).json({ success: true, data: windSensors });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { windSensors, timestamp } = req.body;

        if (!Array.isArray(windSensors) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid WindSensors data format" });
        }

        for (const sensor of windSensors) {
          if (
            typeof sensor.speed !== 'number' ||
            typeof sensor.direction !== 'number'
          ) {
            return res.status(400).json({ success: false, message: "Invalid wind sensor object format" });
          }
        }

        const newWindSensors = new WindSensors({ windSensors, timestamp });
        await newWindSensors.save();

        res.status(201).json({ success: true, message: "WindSensors data stored", data: newWindSensors });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
