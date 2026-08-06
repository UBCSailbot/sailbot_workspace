import BaseSaga from '@/utils/BaseSaga';
import TempSensorsActions from './TempSensorsActions';
import { call, delay, put } from 'redux-saga/effects';
import { TempSensors } from './TempSensorsTypes';
import { TempSensorsService } from './TempSensorsService';

export default class TempSensorsSagas extends BaseSaga {
  *[TempSensorsActions.POLL_TEMPSENSORS]() {
    while (true) {
      try {
        const tempSensors: TempSensors[] = yield call(
          TempSensorsService.getTempSensors,
        );
        if (tempSensors.length > 0) {
          yield put({
            type: TempSensorsActions.REQUEST_TEMPSENSORS_SUCCESS,
            payload: tempSensors,
          });
        }
      } catch (e) {
        yield put({
          type: TempSensorsActions.REQUEST_TEMPSENSORS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
