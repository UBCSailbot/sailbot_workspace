import { expect } from 'chai';
import { api } from '../shared/classes/api';
import { Given, Then } from '@cucumber/cucumber';
import GPS from '@/models/GPS';
import ConnectMongoDB from '@/lib/mongodb';
import AISShips from '@/models/AISShips';
import GlobalPath from '@/models/GlobalPath';
import LocalPath from '@/models/LocalPath';
import Batteries from '@/models/Batteries';
import WindSensors from '@/models/WindSensors';
import TempSensors from '@/models/TempSensors';
import PhSensors from '@/models/PhSensors';
import SalinitySensors from '@/models/SalinitySensors';

Given('I clear the database', async function () {
  const db = await ConnectMongoDB();
  await GPS.deleteMany();
  await AISShips.deleteMany();
  await GlobalPath.deleteMany();
  await LocalPath.deleteMany();
  await Batteries.deleteMany();
  await WindSensors.deleteMany();
  await TempSensors.deleteMany();
  await PhSensors.deleteMany();
  await SalinitySensors.deleteMany();
});

Given('I insert GPS data into the database', async function () {
  const gpsData = {
    latitude: 49.2243,
    longitude: 4.5552,
    speed: 30,
    heading: 45,
  };
  await GPS.create(gpsData);
});

Given('I insert AISShips data into the database', async function () {
  const aisshipData = {
    ships: [
      {
        id: 0,
        latitude: 49.3481,
        longitude: -123.6096,
        cog: -120,
        rot: 50,
        sog: 12.5,
        width: 10,
        length: 80,
      },
      {
        id: 1,
        latitude: 49.4567,
        longitude: -123.3729,
        cog: 75,
        rot: 22,
        sog: 18.2,
        width: 20,
        length: 120,
      },
      {
        id: 2,
        latitude: 49.1728,
        longitude: -123.4578,
        cog: 15,
        rot: 80,
        sog: 20,
        width: 30,
        length: 200,
      },
    ],
  };
  await AISShips.create(aisshipData);
});

Given('I insert GlobalPath data into the database', async function () {
  const globalPathData = {
    waypoints: [
      {
        latitude: 49.37614179786771,
        longitude: -123.27376619978901,
      },
      {
        latitude: 49.37711663428484,
        longitude: -123.27156381625609,
      },
      {
        latitude: 49.378315644557176,
        longitude: -123.27180418927239,
      },
      {
        latitude: 49.381465588831524,
        longitude: -123.27254420646906,
      },
      {
        latitude: 49.3839035958063,
        longitude: -123.2730793585836,
      },
      {
        latitude: 49.38650818896502,
        longitude: -123.27564156703514,
      },
      {
        latitude: 49.38625857180026,
        longitude: -123.28177300276381,
      },
      {
        latitude: 49.382587584844835,
        longitude: -123.29247537578034,
      },
      {
        latitude: 49.37750287441669,
        longitude: -123.29684958339224,
      },
      {
        latitude: 49.37046373776872,
        longitude: -123.3022011892728,
      },
      {
        latitude: 49.362482757047864,
        longitude: -123.30864508742094,
      },
      {
        latitude: 49.35300923158242,
        longitude: -123.31705995027019,
      },
      {
        latitude: 49.34650411159584,
        longitude: -123.3237483126415,
      },
      {
        latitude: 49.34356040541922,
        longitude: -123.34073692035749,
      },
      {
        latitude: 49.342421649614984,
        longitude: -123.34839354509414,
      },
      {
        latitude: 49.34175775635472,
        longitude: -123.35453636335373,
      },
    ],
  };
  await GlobalPath.create(globalPathData);
});

Given('I insert LocalPath data into the database', async function () {
  const localPathData = {
    waypoints: [
      {
        latitude: 49.34356040541922,
        longitude: -123.34073692035749,
      },
      {
        latitude: 49.342421649614984,
        longitude: -123.34839354509414,
      },
      {
        latitude: 49.34175775635472,
        longitude: -123.35453636335373,
      },
    ],
  };
  await LocalPath.create(localPathData);
});

Given('I insert Batteries data into the database', async function () {
  const batteriesData = {
    batteries: [
      {
        voltage: -3.33,
        current: -3.33,
      },
      {
        voltage: 3.33,
        current: 3.33,
      },
    ],
  };
  await Batteries.create(batteriesData);
});

Given('I insert WindSensors data into the database', async function () {
  const windSensorsData = {
    windSensors: [
      {
        speed: 1.11,
        direction: 1,
      },
      {
        speed: 2.22,
        direction: 2,
      },
    ],
  };
  await WindSensors.create(windSensorsData);
});

Given('I insert TempSensors data into the database', async function () {
  const tempSensorsData = {
    tempSensors: [
      {
        temperature: 17.3,
      },
    ],
  };
  await TempSensors.create(tempSensorsData);
});

Given('I insert PhSensors data into the database', async function () {
  const phSensorsData = {
    phSensors: [
      {
        ph: 6.95,
      },
    ],
  };
  await PhSensors.create(phSensorsData);
});

Given('I insert SalinitySensors data into the database', async function () {
  const salinitySensorsData = {
    salinitySensors: [
      {
        salinity: 21000.5,
      },
    ],
  };
  await SalinitySensors.create(salinitySensorsData);
});

Then('the response data matches the data in the database', async function () {
  let apiResponseData_GPS;
  let databaseData_GPS;

  databaseData_GPS = await GPS.find({}).then(function (gps) {
    let transformedGPS = gps.map((data) => data.toJSON());
    return transformedGPS;
  });

  apiResponseData_GPS = api.response.data.data[0];

  const propertiesToCompare = Object.keys(apiResponseData_GPS);

  for (const property of propertiesToCompare) {
    expect(apiResponseData_GPS[property]).to.equal(
      (databaseData_GPS[0] as Record<string, unknown>)[property],
      `Data in the response does not match data in the database for property: ${property}`,
    );
  }
});

Then(
  'the response data matches the aisship data in the database',
  async function () {
    let apiResponseData_AISShips;
    let databaseData_AISShips;

    databaseData_AISShips = await AISShips.find({}).then(function (aisships) {
      let transformedAISShips = aisships.map((data) => data.toJSON());
      return transformedAISShips;
    });

    for (let i = 0; i < 3; i++) {
      apiResponseData_AISShips = api.response.data.data[0].ships[i];

      const propertiesToCompare = Object.keys(apiResponseData_AISShips);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_AISShips[property]).to.equal(
          (databaseData_AISShips[0].ships[i] as Record<string, unknown>)[
            property
          ],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the GlobalPath data in the database',
  async function () {
    let apiResponseData_GlobalPath;
    let databaseData_GlobalPath;

    databaseData_GlobalPath = await GlobalPath.find({}).then(
      function (globalpath) {
        let transformedGlobalPath = globalpath.map((data) => data.toJSON());
        return transformedGlobalPath;
      },
    );

    for (let i = 0; i < 16; i++) {
      apiResponseData_GlobalPath = api.response.data.data[0].waypoints[i];

      const propertiesToCompare = Object.keys(apiResponseData_GlobalPath);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_GlobalPath[property]).to.equal(
          (databaseData_GlobalPath[0].waypoints[i] as Record<string, unknown>)[
            property
          ],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the LocalPath data in the database',
  async function () {
    let apiResponseData_LocalPath;
    let databaseData_LocalPath;

    databaseData_LocalPath = await LocalPath.find({}).then(
      function (localpath) {
        let transformedLocalPath = localpath.map((data) => data.toJSON());
        return transformedLocalPath;
      },
    );

    for (let i = 0; i < 3; i++) {
      apiResponseData_LocalPath = api.response.data.data[0].waypoints[i];

      const propertiesToCompare = Object.keys(apiResponseData_LocalPath);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_LocalPath[property]).to.equal(
          (databaseData_LocalPath[0].waypoints[i] as Record<string, unknown>)[
            property
          ],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the Batteries data in the database',
  async function () {
    let apiResponseData_Batteries;
    let databaseData_Batteries;

    databaseData_Batteries = await Batteries.find({}).then(
      function (batteries) {
        let transformedBatteries = batteries.map((data) => data.toJSON());
        return transformedBatteries;
      },
    );

    for (let i = 0; i < 2; i++) {
      apiResponseData_Batteries = api.response.data.data[0].batteries[i];

      const propertiesToCompare = Object.keys(apiResponseData_Batteries);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_Batteries[property]).to.equal(
          (databaseData_Batteries[0].batteries[i] as Record<string, unknown>)[
            property
          ],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the WindSensors data in the database',
  async function () {
    let apiResponseData_WindSensors;
    let databaseData_WindSensors;

    databaseData_WindSensors = await WindSensors.find({}).then(
      function (windsensors) {
        let transformedWindSensors = windsensors.map((data) => data.toJSON());
        return transformedWindSensors;
      },
    );

    for (let i = 0; i < 2; i++) {
      apiResponseData_WindSensors = api.response.data.data[0].windSensors[i];

      const propertiesToCompare = Object.keys(apiResponseData_WindSensors);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_WindSensors[property]).to.equal(
          (
            databaseData_WindSensors[0].windSensors[i] as Record<
              string,
              unknown
            >
          )[property],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the TempSensors data in the database',
  async function () {
    let apiResponseData_TempSensors;
    let databaseData_TempSensors;

    databaseData_TempSensors = await TempSensors.find({}).then(
      function (tempsensors) {
        let transformedTempSensors = tempsensors.map((data) => data.toJSON());
        return transformedTempSensors;
      },
    );

    for (let i = 0; i < 1; i++) {
      apiResponseData_TempSensors = api.response.data.data[0].tempSensors[i];

      const propertiesToCompare = Object.keys(apiResponseData_TempSensors);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_TempSensors[property]).to.equal(
          (
            databaseData_TempSensors[0].tempSensors[i] as Record<
              string,
              unknown
            >
          )[property],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the PhSensors data in the database',
  async function () {
    let apiResponseData_PhSensors;
    let databaseData_PhSensors;

    databaseData_PhSensors = await PhSensors.find({}).then(
      function (phsensors) {
        let transformedPhSensors = phsensors.map((data) => data.toJSON());
        return transformedPhSensors;
      },
    );

    for (let i = 0; i < 1; i++) {
      apiResponseData_PhSensors = api.response.data.data[0].phSensors[i];

      const propertiesToCompare = Object.keys(apiResponseData_PhSensors);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_PhSensors[property]).to.equal(
          (databaseData_PhSensors[0].phSensors[i] as Record<string, unknown>)[
            property
          ],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);

Then(
  'the response data matches the SalinitySensors data in the database',
  async function () {
    let apiResponseData_SalinitySensors;
    let databaseData_SalinitySensors;

    databaseData_SalinitySensors = await SalinitySensors.find({}).then(
      function (salinitysensors) {
        let transformedSalinitySensors = salinitysensors.map((data) =>
          data.toJSON(),
        );
        return transformedSalinitySensors;
      },
    );

    for (let i = 0; i < 1; i++) {
      apiResponseData_SalinitySensors =
        api.response.data.data[0].salinitySensors[i];

      const propertiesToCompare = Object.keys(apiResponseData_SalinitySensors);

      for (const property of propertiesToCompare) {
        expect(apiResponseData_SalinitySensors[property]).to.equal(
          (
            databaseData_SalinitySensors[0].salinitySensors[i] as Record<
              string,
              unknown
            >
          )[property],
          `Data in the response does not match data in the database for property: ${property}`,
        );
      }
    }
  },
);
