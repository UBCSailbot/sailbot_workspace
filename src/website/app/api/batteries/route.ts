import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import Batteries from '@/models/Batteries';

export async function GET() {
  try {
    await ConnectMongoDB();

    const batteries = await Batteries.find({}).select({
      'batteries._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: batteries });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
