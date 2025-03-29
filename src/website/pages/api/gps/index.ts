import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import GPS from '@/models/GPS';
import { GPS as GPSDocument } from '@/stores/GPS/GPSTypes';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const gps: GPSDocument[] = await GPS.find({}).select('-_id -__v');
        res.status(200).json({ success: true, data: gps });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        let gpsData = req.body;

        if (!Array.isArray(gpsData)) {
          gpsData = [gpsData];
        }

        for (const data of gpsData) {
          const { latitude, longitude, speed, heading, timestamp } = data;
          if (
            typeof latitude !== 'number' ||
            typeof longitude !== 'number' ||
            typeof speed !== 'number' ||
            typeof heading !== 'number' ||
            typeof timestamp !== 'string'
          ) {
            return res.status(400).json({ success: false, message: "Invalid GPS data format" });
          }
        }

        const newGPSData = await GPS.insertMany(gpsData);

        res.status(201).json({ success: true, message: "GPS data stored", data: newGPSData });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'DELETE':
      try {
        await GPS.deleteMany({});
        res.status(200).json({ success: true, message: "All GPS data cleared" });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
