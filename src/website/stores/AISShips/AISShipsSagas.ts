import BaseSaga from '@/utils/BaseSaga';
import AISShipsActions from './AISShipsActions';
import { call, delay, put } from 'redux-saga/effects';
import { AISShips } from './AISShipsTypes';
import { AISShipsService } from './AISShipsService';

export default class AISShipsSagas extends BaseSaga {
  *[AISShipsActions.POLL_AISSHIPS]() {
    while (true) {
      try {
        const aisShips: AISShips[] = yield call(AISShipsService.getAISShips);
        if (aisShips.length > 0) {
          const latestAISShip = aisShips[aisShips.length - 1];
          yield put({
            type: AISShipsActions.REQUEST_AISSHIPS_SUCCESS,
            payload: latestAISShip,
          });
        }
      } catch (e) {
        yield put({
          type: AISShipsActions.REQUEST_AISSHIPS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
