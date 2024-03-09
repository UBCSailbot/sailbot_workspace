import BaseSaga from '@/utils/BaseSaga';
import WindSensorsAction from './WindSensorsActions';
import { call, delay, put } from 'redux-saga/effects';
import { WindSensor } from './WindSensorsTypes';
import { WindSensorsService } from './WindSensorsService';

export default class WindSensorsSagas extends BaseSaga {
  *[WindSensorsAction.POLL_WINDSENSORS]() {
    while (true) {
      try {
        const windSensors: WindSensor[] = yield call(
          WindSensorsService.getWindSensors,
        );
        if (windSensors.length > 0) {
          yield put({
            type: WindSensorsAction.REQUEST_WINDSENSORS_SUCCESS,
            payload: windSensors,
          });
        }
      } catch (e) {
        yield put({
          type: WindSensorsAction.REQUEST_WINDSENSORS_FAILURE,
          error: (e as Error).message,
        });
      }
      // Poll every minute. Adjust the time as needed.
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
