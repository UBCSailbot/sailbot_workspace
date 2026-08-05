import { getApiUrl } from '@/lib/apiUrl';

/**
 * Defines all saga methods to make requests to the TempSensors interface.
 */
export const TempSensorsService = {
  *getTempSensors(): Generator<any, any, any> {
    let isError = false;

    return yield fetch(getApiUrl('/api/temp-sensors'), {
      method: 'GET',
    })
      .then((response) => {
        isError = response.status != 200;
        return response;
      })
      .then((response) => response.json())
      .then((json) => {
        if (isError) {
          throw new Error(json.error);
        }
        return json.data;
      });
  },
};
