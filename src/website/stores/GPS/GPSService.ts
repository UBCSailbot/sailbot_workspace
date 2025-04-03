/**
 * Defines all saga methods to make requests to the GPS interface.
 */
export const GPSService = {
  *getGPS(): Generator<any, any, any> {
    let isError = false;

    return yield fetch(
      `/api/gps`,
      {
        method: 'GET',
      },
    )
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
