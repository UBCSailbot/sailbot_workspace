import BaseReducer from '@/utils/BaseReducer';
import AISShipsActions from './AISShipsActions';
import { AISShipsState } from './AISShipsTypes';
import { AnyAction } from 'redux';

export default class AISShipsReducer extends BaseReducer {
  initialState: AISShipsState = {
    data: {
      ships: [],
    },
    error: null,
  };

  [AISShipsActions.REQUEST_AISSHIPS_SUCCESS](
    state: AISShipsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      data: action.payload,
    };
  }

  [AISShipsActions.REQUEST_AISSHIPS_FAILURE](
    state: AISShipsState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
