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
        const { latitude, longitude, speed, heading, timestamp } = req.body;

        if (
          typeof latitude !== 'number' ||
          typeof longitude !== 'number' ||
          typeof speed !== 'number' ||
          typeof heading !== 'number' ||
          typeof timestamp !== 'string'
        ) {
          return res.status(400).json({ success: false, message: "Invalid GPS data format" });
        }

        const newGPS = new GPS({ latitude, longitude, speed, heading, timestamp });
        await newGPS.save();

        res.status(201).json({ success: true, message: "GPS data stored", data: newGPS });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
