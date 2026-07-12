import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import SalinitySensors from '@/models/SalinitySensors';

export async function GET() {
  try {
    await ConnectMongoDB();

    const salinitySensors = await SalinitySensors.find({}).select({
      'salinitySensors._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: salinitySensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
