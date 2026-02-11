import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphsState, Layout } from './GraphsTypes';
import { AnyAction } from 'redux';
import { initSessionStorageData } from '@/utils/SessionStorage';

export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    layout: initSessionStorageData('Graph Layout', [
      'GPS',
      'BatteriesVoltage',
      'BatteriesCurrent',
      'WindSensors',
    ]) as Layout,
    error: null,
  };

  [GraphsActions.REARRANGE_GRAPHS_SUCCESS](
    state: GraphsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      layout: action.payload,
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
