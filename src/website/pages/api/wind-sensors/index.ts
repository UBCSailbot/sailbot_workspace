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
