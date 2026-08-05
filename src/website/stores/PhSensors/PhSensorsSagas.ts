import BaseSaga from '@/utils/BaseSaga';
import PhSensorsActions from './PhSensorsActions';
import { call, delay, put } from 'redux-saga/effects';
import { PhSensors } from './PhSensorsTypes';
import { PhSensorsService } from './PhSensorsService';

export default class PhSensorsSagas extends BaseSaga {
  *[PhSensorsActions.POLL_PHSENSORS]() {
    while (true) {
      try {
        const phSensors: PhSensors[] = yield call(
          PhSensorsService.getPhSensors,
        );
        if (phSensors.length > 0) {
          yield put({
            type: PhSensorsActions.REQUEST_PHSENSORS_SUCCESS,
            payload: phSensors,
          });
        }
      } catch (e) {
        yield put({
          type: PhSensorsActions.REQUEST_PHSENSORS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
