import BaseSaga from '@/utils/BaseSaga';
import GenericSensorsActions from './GenericSensorsActions';
import { call, delay, put } from 'redux-saga/effects';
import { GenericSensor } from './GenericSensorsTypes';
import { GenericSensorsService } from './GenericSensorsService';

export default class GenericSensorsSagas extends BaseSaga {
  *[GenericSensorsActions.POLL_GENERICSENSORS]() {
    while (true) {
      try {
        const genericSensors: GenericSensor[] = yield call(
          GenericSensorsService.getGenericSensors,
        );
        if (genericSensors.length > 0) {
          yield put({
            type: GenericSensorsActions.REQUEST_GENERICSENSORS_SUCCESS,
            payload: genericSensors,
          });
        }
      } catch (e) {
        yield put({
          type: GenericSensorsActions.REQUEST_GENERICSENSORS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
