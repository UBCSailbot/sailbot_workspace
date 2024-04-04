import BaseSaga from '@/utils/BaseSaga';
import GraphsActions from './GraphsActions';
import { put, takeLatest } from 'redux-saga/effects';

export default class GraphsSagas extends BaseSaga {
    *rearrangeGraphs(action) {
        try {
            yield put({
                type: GraphsActions.REARRANGE_GRAPHS_SUCCESS,
                payload: action.payload,
            })
        } catch (e) {
            yield put ({
                type: GraphsActions.REARRANGE_GRAPHS_FAILURE,
                error: e.message,
            })
        }
    }

    *[GraphsActions.REARRANGE_GRAPHS]() {
        yield takeLatest(GraphsActions.REARRANGE_GRAPHS, this.rearrangeGraphs)
    }
}
