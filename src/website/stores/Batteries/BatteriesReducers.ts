import BaseReducer from '@/utils/BaseReducer';
import BatteriesActions from './BatteriesActions';
import { Batteries, BatteriesState } from './BatteriesTypes';
import { AnyAction } from 'redux';

export default class BatteriesReducer extends BaseReducer {
  initialState: BatteriesState = {
    data: [] as Batteries[],
    error: null,
  };

  [BatteriesActions.REQUEST_BATTERIES_SUCCESS](
    state: BatteriesState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [BatteriesActions.REQUEST_BATTERIES_FAILURE](
    state: BatteriesState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
