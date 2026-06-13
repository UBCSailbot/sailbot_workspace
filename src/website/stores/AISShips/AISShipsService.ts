import { getApiUrl } from '@/lib/apiUrl';

/**
 * Defines all saga methods to make requests to the AISShips interface.
 */
export const AISShipsService = {
  *getAISShips(): Generator<any, any, any> {
    let isError = false;

    // Only the newest snapshot is rendered, so don't download the full history.
    return yield fetch(
      getApiUrl('/api/aisships?latest=true'),
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
