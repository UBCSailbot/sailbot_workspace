import BaseReducer from '@/utils/BaseReducer';
import GlobalPathActions from './GlobalPathActions';
import { GlobalPathState } from './GlobalPathTypes';
import { AnyAction } from 'redux';

export default class GlobalPathReducer extends BaseReducer {
  initialState: GlobalPathState = {
    data: {
      waypoints: [],
    },
    error: null,
  };

  [GlobalPathActions.REQUEST_GLOBALPATH_SUCCESS](
    state: GlobalPathState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [GlobalPathActions.REQUEST_GLOBALPATH_FAILURE](
    state: GlobalPathState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
