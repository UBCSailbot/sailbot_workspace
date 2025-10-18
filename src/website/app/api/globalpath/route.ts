import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import GlobalPath from '@/models/GlobalPath';
import { GlobalPath as GlobalPathDocument } from '@/stores/GlobalPath/GlobalPathTypes';

export async function GET() {
  try {
    await ConnectMongoDB();

    const gPath: GlobalPathDocument[] = await GlobalPath.find({}).select({
      'waypoints._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: gPath });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
