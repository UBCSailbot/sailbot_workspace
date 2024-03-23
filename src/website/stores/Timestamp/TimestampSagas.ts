import BaseSaga from '@/utils/BaseSaga';
import TimestampActions from './TimestampActions';
import { call, delay, put, takeLatest } from 'redux-saga/effects';
import { Timestamp } from './TimestampTypes';

export default class TimestampSagas extends BaseSaga {
  *replaceTimestamp(action) {
    try {
      yield put({
        type: TimestampActions.REQUEST_TIMESTAMP_SUCCESS,
        payload: action.payload,
      });
    } catch (e) {
      yield put({
        type: TimestampActions.REQUEST_TIMESTAMP_FAILURE,
        error: e.message,
      });
    }
  }

  *[TimestampActions.TIMESTAMP]() {
    yield takeLatest(TimestampActions.TIMESTAMP, this.replaceTimestamp);
  }
}
