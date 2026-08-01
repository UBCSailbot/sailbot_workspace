import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import PhSensors from '@/models/PhSensors';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    const limitParam = new URL(request.url).searchParams.get('limit');
    const limit = limitParam ? parseInt(limitParam, 10) : 0;

    let query = PhSensors.find({}).select({
      'phSensors._id': 0,
      _id: 0,
      __v: 0,
    });
    if (limit > 0) {
      query = query.sort({ _id: -1 }).limit(limit);
    }

    const phSensors = await query.lean();
    if (limit > 0) phSensors.reverse();

    return NextResponse.json({ success: true, data: phSensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
