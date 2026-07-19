import BaseReducer from '@/utils/BaseReducer';
import TempSensorsActions from './TempSensorsActions';
import { TempSensorsState, TempSensors } from './TempSensorsTypes';
import { AnyAction } from 'redux';

export default class TempSensorsReducer extends BaseReducer {
  initialState: TempSensorsState = {
    data: [] as TempSensors[],
    error: null,
  };

  [TempSensorsActions.REQUEST_TEMPSENSORS_SUCCESS](
    state: TempSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [TempSensorsActions.REQUEST_TEMPSENSORS_FAILURE](
    state: TempSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
