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
        res
          .status(400)
          .json({ success: false, message: (error as Error).message });
      }
      break;
    default:
      res.status(400).json({ success: false });
      break;
  }
}
