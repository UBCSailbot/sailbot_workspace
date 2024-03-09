import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphsState } from './BatteriesTypes';
import { AnyAction } from 'redux';

export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    order: ["1", "2", "3", "4"],
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
