import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import WindSensors from '@/models/WindSensors';
import { decimal2JSON } from '@/models/helper/parser';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    const limitParam = new URL(request.url).searchParams.get('limit');
    const limit = limitParam ? parseInt(limitParam, 10) : 0;

    let query = WindSensors.find({}).select({
      'windSensors._id': 0,
      _id: 0,
      __v: 0,
    });
    if (limit > 0) {
      query = query.sort({ _id: -1 }).limit(limit);
    }

    const windSensors = await query.lean();
    if (limit > 0) windSensors.reverse();
    windSensors.forEach((doc) => decimal2JSON(doc));

    return NextResponse.json({ success: true, data: windSensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
