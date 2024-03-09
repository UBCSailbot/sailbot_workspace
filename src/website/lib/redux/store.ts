/* Core */
import { applyMiddleware } from 'redux';
import {
  configureStore,
  type ThunkAction,
  type Action,
} from '@reduxjs/toolkit';
import {
  useSelector as useReduxSelector,
  useDispatch as useReduxDispatch,
  type TypedUseSelectorHook,
} from 'react-redux';
import createSagaMiddleware from 'redux-saga';

/* Instruments */
import { rootReducer } from './rootReducer';
import { middleware } from './middleware';
import { rootSaga } from './rootSaga';

const createReduxStore = () => {
  const sagaMiddleWare = createSagaMiddleware();
  const store = configureStore({
    reducer: rootReducer(),
    middleware: (getDefaultMiddleware) =>
      getDefaultMiddleware().concat([sagaMiddleWare, ...middleware]),
    enhancers: [applyMiddleware(sagaMiddleWare)],
  });

  sagaMiddleWare.run(rootSaga);

  return store;
};

export const reduxStore = createReduxStore();

export const useDispatch = () => useReduxDispatch<ReduxDispatch>();
export const useSelector: TypedUseSelectorHook<ReduxState> = useReduxSelector;

/* Types */
export type ReduxStore = typeof reduxStore;
export type ReduxState = ReturnType<typeof reduxStore.getState>;
export type ReduxDispatch = typeof reduxStore.dispatch;
export type ReduxThunkAction<ReturnType = void> = ThunkAction<
  ReturnType,
  ReduxState,
  unknown,
  Action
>;
