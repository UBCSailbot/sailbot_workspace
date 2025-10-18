import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphsState } from './GraphsTypes';
import { AnyAction } from 'redux';
import { initSessionStorageData } from '@/utils/SessionStorage';

export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    order: initSessionStorageData('Current Order', [
      'GPS',
      'BatteriesVoltage',
      'BatteriesCurrent',
      'WindSensors',
    ]),
    error: null,
  };

  [GraphsActions.REARRANGE_GRAPHS_SUCCESS](
    state: GraphsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      order: action.payload,
    };
  }

  [GraphsActions.REARRANGE_GRAPHS_FAILURE](
    state: GraphsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
