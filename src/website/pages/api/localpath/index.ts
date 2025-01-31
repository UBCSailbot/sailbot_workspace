import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import LocalPath from '@/models/LocalPath';
import { LocalPath as LocalPathDocument } from '@/stores/LocalPath/LocalPathTypes';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const localPath: LocalPathDocument[] = await LocalPath.find({}).select({
          'waypoints._id': 0,
          _id: 0,
          __v: 0,
        });
        res.status(200).json({ success: true, data: localPath });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { waypoints, timestamp } = req.body;

        if (!Array.isArray(waypoints) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid LocalPath data format" });
        }

        for (const waypoint of waypoints) {
          if (
            typeof waypoint.latitude !== 'number' ||
            typeof waypoint.longitude !== 'number'
          ) {
            return res.status(400).json({ success: false, message: "Invalid waypoint object format" });
          }
        }

        const newLocalPath = new LocalPath({ waypoints, timestamp });
        await newLocalPath.save();

        res.status(201).json({ success: true, message: "LocalPath data stored", data: newLocalPath });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
