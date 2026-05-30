import BaseReducer from '@/utils/BaseReducer';
import WindSensorsAction from './WindSensorsActions';
import { WindSensorsState, WindSensors } from './WindSensorsTypes';
import { AnyAction } from 'redux';

export default class WindSensorsReducer extends BaseReducer {
  initialState: WindSensorsState = {
    data: [] as WindSensors[],
    error: null,
  };

  [WindSensorsAction.REQUEST_WINDSENSORS_SUCCESS](
    state: WindSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [WindSensorsAction.REQUEST_WINDSENSORS_FAILURE](
    state: WindSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
