import BaseSaga from '@/utils/BaseSaga';
import BatteriesActions from './BatteriesActions';
import { call, delay, put } from 'redux-saga/effects';
import { Battery } from './BatteriesTypes';
import { BatteriesService } from './BatteriesService';

export default class BatteriesSagas extends BaseSaga {
  *[BatteriesActions.POLL_BATTERIES]() {
    while (true) {
      try {
        const batteries: Battery[] = yield call(BatteriesService.getBatteries);
        if (batteries.length > 0) {
          yield put({
            type: BatteriesActions.REQUEST_BATTERIES_SUCCESS,
            payload: batteries,
          });
        }
      } catch (e) {
        yield put({
          type: BatteriesActions.REQUEST_BATTERIES_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
