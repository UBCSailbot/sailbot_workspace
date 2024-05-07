import BaseSaga from '@/utils/BaseSaga';
import GPSActions from './GPSActions';
import { call, delay, put } from 'redux-saga/effects';
import { GPS } from './GPSTypes';
import { GPSService } from './GPSService';

export default class GPSSagas extends BaseSaga {
  *[GPSActions.POLL_GPS]() {
    while (true) {
      try {
        const data: GPS[] = yield call(GPSService.getGPS);
        if (data.length > 0) {
          yield put({ type: GPSActions.REQUEST_GPS_SUCCESS, payload: data });
        }
      } catch (e) {
        yield put({
          type: GPSActions.REQUEST_GPS_FAILURE,
          error: (e as Error).message,
        });
      }
      // @ts-ignore
      yield delay(process.env.NEXT_PUBLIC_POLLING_TIME_MS);
    }
  }
}
