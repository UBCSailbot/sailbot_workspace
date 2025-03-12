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
        let globalPathData = req.body;

        if (!Array.isArray(globalPathData)) {
          globalPathData = [globalPathData];
        }

        const processedPaths = globalPathData.map(entry => {
          if (!Array.isArray(entry.waypoints)) {
            throw new Error("Invalid GlobalPath data format");
          }

          for (const waypoint of entry.waypoints) {
            if (
              typeof waypoint.latitude !== 'number' ||
              typeof waypoint.longitude !== 'number'
            ) {
              throw new Error("Invalid waypoint object format");
            }
          }

          return {
            waypoints: entry.waypoints,
            timestamp: entry.timestamp || new Date().toISOString()
          };
        });

        const newGlobalPaths = await GlobalPath.insertMany(processedPaths);

        res.status(201).json({ success: true, message: "GlobalPath data stored", data: newGlobalPaths });
      } catch (error) {
        res.status(400).json({ success: false, message: error.message });
      }
      break;

    case 'DELETE':
      try {
        await GlobalPath.deleteMany({});
        res.status(200).json({ success: true, message: "All GlobalPath data cleared" });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
