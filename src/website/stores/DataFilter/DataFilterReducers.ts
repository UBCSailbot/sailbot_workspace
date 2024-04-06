import BaseReducer from '@/utils/BaseReducer';
import DataFilterActions from './DataFilterActions';
import { DataFilterState } from './DataFilterTypes';
import { AnyAction } from 'redux';

export default class DataFilterReducer extends BaseReducer {
  initialState: DataFilterState = {
    timestamps: {
      startDate: null,
      endDate: null,
    },
    error: null,
  };

  [DataFilterActions.SET_TIMESTAMP_SUCCESS](
    state: DataFilterState,
    action: AnyAction,
  ) {
    return {
      ...state,
      timestamps: action.payload,
    };
  }

  [DataFilterActions.SET_TIMESTAMP_FAILURE](
    state: DataFilterState,
    action: AnyAction,
  ) {
    return {
      ...state,
      error: action.error,
    };
  }
}
