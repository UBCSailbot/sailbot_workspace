import BaseReducer from '@/utils/BaseReducer';
import GenericSensorActions from './GenericSensorsActions';
import { GenericSensorsState } from './GenericSensorsTypes';
import { AnyAction } from 'redux';

export default class GenericSensorsReducer extends BaseReducer {
  initialState: GenericSensorsState = {
    data: {
      genericSensors: [],
    },
    error: null,
  };

  [GenericSensorActions.REQUEST_GENERICSENSORS_SUCCESS](
    state: GenericSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [GenericSensorActions.REQUEST_GENERICSENSORS_FAILURE](
    state: GenericSensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.error,
    };
  }
}
