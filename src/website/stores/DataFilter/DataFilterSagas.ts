import BaseSaga from '@/utils/BaseSaga';
import DataFilterActions from './DataFilterActions';
import { put, takeLatest } from 'redux-saga/effects';

export default class DataFilterSagas extends BaseSaga {
  *replaceTimestamp(action) {
    try {
      yield put({
        type: DataFilterActions.SET_TIMESTAMP_SUCCESS,
        payload: action.payload,
      });
    } catch (e) {
      yield put({
        type: DataFilterActions.SET_TIMESTAMP_FAILURE,
        error: e.message,
      });
    }
  }

  *[DataFilterActions.SET_TIMESTAMP]() {
    yield takeLatest(DataFilterActions.SET_TIMESTAMP, this.replaceTimestamp);
  }
}
