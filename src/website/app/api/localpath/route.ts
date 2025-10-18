import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import LocalPath from '@/models/LocalPath';
import { LocalPath as LocalPathDocument } from '@/stores/LocalPath/LocalPathTypes';

export async function GET() {
  try {
    await ConnectMongoDB();

    const localPath: LocalPathDocument[] = await LocalPath.find({}).select({
      'waypoints._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: localPath });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
