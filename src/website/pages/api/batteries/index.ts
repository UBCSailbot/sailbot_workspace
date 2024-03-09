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
