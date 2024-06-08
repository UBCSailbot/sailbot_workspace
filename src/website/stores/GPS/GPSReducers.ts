import BaseReducer from '@/utils/BaseReducer';
import GPSActions from './GPSActions';
import { GPSState } from './GPSTypes';
import { AnyAction } from 'redux';

export default class GPSReducer extends BaseReducer {
  initialState: GPSState = {
    data: [
      {
        latitude: 49.37614179786771,
        longitude: -123.27376619978901,
        speed: 0,
        heading: 0,
        timestamp: new Date().toISOString(),
      },
    ],
  };

  [GPSActions.REQUEST_GPS_SUCCESS](state: GPSState, action: AnyAction) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [GPSActions.REQUEST_GPS_FAILURE](state: GPSState, action: AnyAction) {
    return {
      ...state,
      error: action.error,
    };
  }
}
