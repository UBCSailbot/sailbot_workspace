import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import GPS from '@/models/GPS';
import { decimal2JSON } from '@/models/helper/parser';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    const limitParam = new URL(request.url).searchParams.get('limit');
    const limit = limitParam ? parseInt(limitParam, 10) : 0;

    let query = GPS.find({}).select('-_id -__v');
    if (limit > 0) {
      query = query.sort({ _id: -1 }).limit(limit);
    }

    const gps = await query.lean();
    if (limit > 0) gps.reverse();
    gps.forEach((doc) => decimal2JSON(doc));

    return NextResponse.json({ success: true, data: gps });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
