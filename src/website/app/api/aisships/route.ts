import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import AISShips from '@/models/AISShips';
import { AISShips as AISShipsDocument } from '@/stores/AISShips/AISShipsTypes';

export async function GET() {
  try {
    await ConnectMongoDB();

    const aisships: AISShipsDocument[] = await AISShips.find({}).select({
      'ships._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: aisships });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
