import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import GlobalPath from '@/models/GlobalPath';
import { GlobalPath as GlobalPathDocument } from '@/stores/GlobalPath/GlobalPathTypes';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const gPath: GlobalPathDocument[] = await GlobalPath.find({}).select({
          'waypoints._id': 0,
          _id: 0,
          __v: 0,
        });
        res.status(200).json({ success: true, data: gPath });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { waypoints, timestamp } = req.body;

        if (!Array.isArray(waypoints) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid GlobalPath data format" });
        }

        for (const waypoint of waypoints) {
          if (
            typeof waypoint.latitude !== 'number' ||
            typeof waypoint.longitude !== 'number'
          ) {
            return res.status(400).json({ success: false, message: "Invalid waypoint object format" });
          }
        }

        const newGlobalPath = new GlobalPath({ waypoints, timestamp });
        await newGlobalPath.save();

        res.status(201).json({ success: true, message: "GlobalPath data stored", data: newGlobalPath });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
