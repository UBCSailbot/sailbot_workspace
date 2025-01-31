import { NextApiRequest, NextApiResponse } from 'next';
import ConnectMongoDB from '@/lib/mongodb';
import AISShips from '@/models/AISShips';
import { AISShips as AISShipsDocument } from '@/stores/AISShips/AISShipsTypes';

export default async function handler(
  req: NextApiRequest,
  res: NextApiResponse,
) {
  const { method } = req;

  await ConnectMongoDB();

  switch (method) {
    case 'GET':
      try {
        const aisships: AISShipsDocument[] = await AISShips.find({}).select({
          'ships._id': 0,
          _id: 0,
          __v: 0,
        });
        res.status(200).json({ success: true, data: aisships });
      } catch (error) {
        res.status(400).json({ success: false, message: (error as Error).message });
      }
      break;

    case 'POST':
      try {
        const { ships, timestamp } = req.body;

        if (!Array.isArray(ships) || typeof timestamp !== 'string') {
          return res.status(400).json({ success: false, message: "Invalid AISShips data format" });
        }

        for (const ship of ships) {
          if (
            typeof ship.id !== 'number' ||
            typeof ship.latitude !== 'number' ||
            typeof ship.longitude !== 'number' ||
            typeof ship.cog !== 'number' ||
            typeof ship.rot !== 'number' ||
            typeof ship.sog !== 'number' ||
            typeof ship.width !== 'number' ||
            typeof ship.length !== 'number'
          ) {
            return res.status(400).json({ success: false, message: "Invalid ship object format" });
          }
        }

        const newAISShips = new AISShips({ ships, timestamp });
        await newAISShips.save();

        res.status(201).json({ success: true, message: "AISShips data stored", data: newAISShips });
      } catch (error) {
        res.status(500).json({ success: false, message: (error as Error).message });
      }
      break;

    default:
      res.status(405).json({ success: false, message: "Method Not Allowed" });
      break;
  }
}
