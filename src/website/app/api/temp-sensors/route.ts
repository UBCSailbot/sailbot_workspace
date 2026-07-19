import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import TempSensors from '@/models/TempSensors';

export async function GET() {
  try {
    await ConnectMongoDB();

    const tempSensors = await TempSensors.find({}).select({
      'tempSensors._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: tempSensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
