import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import Batteries from '@/models/Batteries';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const batteries = await Batteries.find({}).select({
          'batteries._id': 0,
          _id: 0,
          __v: 0,
        });
        res.status(200).json({ success: true, data: batteries });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { batteries, timestamp } = req.body;

        if (!Array.isArray(batteries) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid Batteries data format" });
        }

        for (const battery of batteries) {
          if (
            typeof battery.voltage !== 'number' ||
            typeof battery.current !== 'number'
          ) {
            return res.status(400).json({ success: false, message: "Invalid battery object format" });
          }
        }

        const newBatteries = new Batteries({ batteries, timestamp });
        await newBatteries.save();

        res.status(201).json({ success: true, message: "Batteries data stored", data: newBatteries });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
