import BaseSaga from '@/utils/BaseSaga';
import SalinitySensorsActions from './SalinitySensorsActions';
import { call, delay, put } from 'redux-saga/effects';
import { SalinitySensors } from './SalinitySensorsTypes';
import { SalinitySensorsService } from './SalinitySensorsService';

export default class SalinitySensorsSagas extends BaseSaga {
  *[SalinitySensorsActions.POLL_SALINITYSENSORS]() {
    while (true) {
      try {
        const salinitySensors: SalinitySensors[] = yield call(
          SalinitySensorsService.getSalinitySensors,
        );
        if (salinitySensors.length > 0) {
          yield put({
            type: SalinitySensorsActions.REQUEST_SALINITYSENSORS_SUCCESS,
            payload: salinitySensors,
          });
        }
      } catch (e) {
        yield put({
          type: SalinitySensorsActions.REQUEST_SALINITYSENSORS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
