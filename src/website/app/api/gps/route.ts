import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import GPS from '@/models/GPS';
import { GPS as GPSDocument } from '@/stores/GPS/GPSTypes';

export async function GET() {
  try {
    await ConnectMongoDB();

    const gps: GPSDocument[] = await GPS.find({}).select('-_id -__v');

    return NextResponse.json({ success: true, data: gps });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
