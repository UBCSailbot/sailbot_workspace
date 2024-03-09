import BaseReducer from '@/utils/BaseReducer';
import LocalPathActions from './LocalPathActions';
import { LocalPathState } from './LocalPathTypes';
import { AnyAction } from 'redux';

export default class LocalPathReducer extends BaseReducer {
  initialState: LocalPathState = {
    data: {
      waypoints: [],
    },
    error: null,
  };

  [LocalPathActions.REQUEST_LOCALPATH_SUCCESS](
    state: LocalPathState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [LocalPathActions.REQUEST_LOCALPATH_FAILURE](
    state: LocalPathState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
