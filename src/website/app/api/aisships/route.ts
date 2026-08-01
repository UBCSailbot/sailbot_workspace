import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import AISShips from '@/models/AISShips';
import { decimal2JSON } from '@/models/helper/parser';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    const params = new URL(request.url).searchParams;
    const latestOnly = params.get('latest') === 'true';
    const limitParam = params.get('limit');
    const limit = latestOnly ? 1 : limitParam ? parseInt(limitParam, 10) : 0;

    let query = AISShips.find({}).select({
      'ships._id': 0,
      _id: 0,
      __v: 0,
    });
    if (limit > 0) {
      query = query.sort({ _id: -1 }).limit(limit);
    }

    const aisships = await query.lean();
    if (limit > 0 && !latestOnly) aisships.reverse();
    aisships.forEach((doc) => decimal2JSON(doc));

    return NextResponse.json({ success: true, data: aisships });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
