import { NextResponse } from 'next/server';
import ConnectMongoDB from '@/lib/mongodb';

import AISShips from '@/models/AISShips';
import Batteries from '@/models/Batteries';
import GenericSensors from '@/models/GenericSensors';
import GlobalPath from '@/models/GlobalPath';
import GPS from '@/models/GPS';
import LocalPath from '@/models/LocalPath';
import WindSensors from '@/models/WindSensors';

type LastUpdatedMap = Record<string, string | null>;

async function getLastUpdated(Model: any): Promise<string | null> {
  const latest = await Model.findOne({})
    .sort({ timestamp: -1 })
    .select({ timestamp: 1, _id: 0 })
    .lean();

  return latest?.timestamp ?? null;
}

export async function GET() {
  try {
    await ConnectMongoDB();

    const data: LastUpdatedMap = {
      GPS: await getLastUpdated(GPS),
      AIS: await getLastUpdated(AISShips),
      GlobalPath: await getLastUpdated(GlobalPath),
      LocalPath: await getLastUpdated(LocalPath),
      Batteries: await getLastUpdated(Batteries),
      WindSensors: await getLastUpdated(WindSensors),
      GenericSensors: await getLastUpdated(GenericSensors),
    };

    return NextResponse.json({ success: true, data });
  } catch (error) {
    return NextResponse.json(
      { success: false, message: (error as Error).message },
      { status: 400 },
    );
  }
}
