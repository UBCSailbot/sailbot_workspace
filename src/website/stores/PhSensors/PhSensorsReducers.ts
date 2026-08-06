import BaseReducer from '@/utils/BaseReducer';
import PhSensorsActions from './PhSensorsActions';
import { PhSensorsState, PhSensors } from './PhSensorsTypes';
import { AnyAction } from 'redux';

export default class PhSensorsReducer extends BaseReducer {
  initialState: PhSensorsState = {
    data: [] as PhSensors[],
    error: null,
  };

  [PhSensorsActions.REQUEST_PHSENSORS_SUCCESS](
    state: PhSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [PhSensorsActions.REQUEST_PHSENSORS_FAILURE](
    state: PhSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
