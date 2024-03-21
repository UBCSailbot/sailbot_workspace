import BaseReducer from '@/utils/BaseReducer';
import TimestampActions from './TimestampActions'
import { TimestampState } from './TimestampTypes';
import { AnyAction } from 'redux'

export default class TimestampReducer extends BaseReducer {
    initialState: TimestampState = {
        timestamp: {
            startDate: null,
            endDate: null
        },
        error: null
    };

    [TimestampActions.REQUEST_TIMESTAMP_SUCCESS](
        state: TimestampState,
        action: AnyAction,
    ) {
        return {
            ...state,
            timestamp: action.payload,
        };
    }

    [TimestampActions.REQUEST_TIMESTAMP_FAILURE](
        state: TimestampState,
        action: AnyAction,
    ) {
        return {
            ...state,
            error: action.error,
        };
    }
}
