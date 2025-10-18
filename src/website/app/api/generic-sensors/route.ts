import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';
import GenericSensors from '@/models/GenericSensors';

export async function GET() {
  try {
    await ConnectMongoDB();

    const genericSensors = await GenericSensors.find({}).select({
      'genericSensors._id': 0,
      _id: 0,
      __v: 0,
    });

    return NextResponse.json({ success: true, data: genericSensors });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
