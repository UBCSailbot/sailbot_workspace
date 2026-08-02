import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import TempSensors from '@/models/TempSensors';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    const limitParam = new URL(request.url).searchParams.get('limit');
    const limit = limitParam ? parseInt(limitParam, 10) : 0;

    let query = TempSensors.find({}).select({
      'tempSensors._id': 0,
      _id: 0,
      __v: 0,
    });
    if (limit > 0) {
      query = query.sort({ _id: -1 }).limit(limit);
    }

    const tempSensors = await query.lean();
    if (limit > 0) tempSensors.reverse();

    return NextResponse.json({ success: true, data: tempSensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
