import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import AISShips from '@/models/AISShips';
import { AISShips as AISShipsDocument } from '@/stores/AISShips/AISShipsTypes';

export async function GET(request: Request) {
  try {
    await ConnectMongoDB();

    // `?latest=true` returns only the newest snapshot. The map poller uses it;
    // the full history stays available (default) for dataset downloads.
    const latestOnly =
      new URL(request.url).searchParams.get('latest') === 'true';

    let query = AISShips.find({});
    if (latestOnly) {
      query = query.sort({ _id: -1 }).limit(1);
    }

    const aisships: AISShipsDocument[] = await query.select({
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
