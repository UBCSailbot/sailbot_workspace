/**
 * Defines all saga methods to make requests to the Batteries interface.
 */
export const BatteriesService = {
  *getBatteries(): Generator<any, any, any> {
    let isError = false;

    return yield fetch(
      `${process.env.NEXT_PUBLIC_SERVER_HOST}:${process.env.NEXT_PUBLIC_SERVER_PORT}/api/batteries`,
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
