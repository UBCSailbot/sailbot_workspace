import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphsState } from './GraphsTypes';
import { AnyAction } from 'redux';
import { initSessionStorageData, saveSessionStorageData} from '@/utils/SessionStorage';
const defaultLayout: GraphsState['layout'] = {
  GPS: 'full',
  BatteriesVoltage: 'half',
  BatteriesCurrent: 'half',
  WindSensors: 'full',
};

export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    order: initSessionStorageData('Current Order', [
      'GPS',
      'BatteriesVoltage',
      'BatteriesCurrent',
      'WindSensors',
    ]),
    layout: initSessionStorageData('Default Layout', defaultLayout) || {
      GPS: 'full',
      BatteriesVoltage: 'half',
      BatteriesCurrent: 'half',
      WindSensors: 'full',
    },
    error: null,
  };

  [GraphsActions.SET_GRAPH_LAYOUT](state: GraphsState, action: AnyAction) {
    const { id, value } = action.payload as { id: keyof GraphsState['layout']; value: 'full' | 'half' };
    return { ...state, layout: { ...state.layout, [id]: value } };
  }


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
