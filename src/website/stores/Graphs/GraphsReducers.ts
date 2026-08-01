import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphId, GraphsState, Layout } from './GraphsTypes';
import { AnyAction } from 'redux';
import { initSessionStorageData } from '@/utils/SessionStorage';
import { mergeMissingGraphIds } from './GraphsLayoutHelpers';

const DEFAULT_LAYOUT: GraphId[] = [
  'GPS',
  'BatteriesVoltage',
  'BatteriesCurrent',
  'WindSensors',
  'Temperature',
  'PH',
  'Salinity',
];

export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    layout: mergeMissingGraphIds(
      initSessionStorageData('Graph Layout', DEFAULT_LAYOUT) as Layout,
      DEFAULT_LAYOUT,
    ),
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
