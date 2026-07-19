import BaseReducer from '@/utils/BaseReducer';
import SalinitySensorsActions from './SalinitySensorsActions';
import { SalinitySensorsState, SalinitySensors } from './SalinitySensorsTypes';
import { AnyAction } from 'redux';

export default class SalinitySensorsReducer extends BaseReducer {
  initialState: SalinitySensorsState = {
    data: [] as SalinitySensors[],
    error: null,
  };

  [SalinitySensorsActions.REQUEST_SALINITYSENSORS_SUCCESS](
    state: SalinitySensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [SalinitySensorsActions.REQUEST_SALINITYSENSORS_FAILURE](
    state: SalinitySensorsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
