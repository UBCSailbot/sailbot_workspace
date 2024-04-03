import BaseReducer from '@/utils/BaseReducer';
import GraphsActions from './GraphsActions';
import { GraphsState } from './GraphsTypes';
import { AnyAction } from 'redux';
import { initSessionStorageData } from '@/utils/SessionStorage'


// const getFromSessionStorage = (key: string) => {
//   console.log(typeof window === 'undefined')
//   // console.log(sessionStorage.getItem(key) === null)
//   console.log(!sessionStorage)
//   // if (typeof window === 'undefined' || sessionStorage.getItem(key) === null || !sessionStorage) {
//   //   console.log("blahb blah blah")
//   //   return ["speed", "battery voltage", "battery current", "wind sensor"]
//   // }
//   // console.log(sessionStorage.getItem(key))
//   // return sessionStorage.getItem(key)
//   // return ["speed", "battery voltage", "battery current", "wind sensor"]
// }
export default class GraphsReducer extends BaseReducer {
  initialState: GraphsState = {
    order: initSessionStorageData('Current Order', ["speed", "battery voltage", "battery current", "wind sensor"]),
    // order: ["speed", "battery voltage", "battery current", "wind sensor"],
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
