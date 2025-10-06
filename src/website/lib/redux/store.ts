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
import { Theme } from './theme/themeSlice';

const getPreloadedTheme = () => {
  try {
    const stored =
      typeof window !== 'undefined' ? localStorage.getItem('theme') : null;
    if (stored === 'dark') return Theme.Dark;
    if (stored === 'light') return Theme.Light;
  } catch {}
  if (typeof window !== 'undefined' && window.matchMedia) {
    return window.matchMedia('(prefers-color-scheme: dark)').matches
      ? Theme.Dark
      : Theme.Light;
  }
  return Theme.Dark;
};

const createReduxStore = () => {
  const sagaMiddleWare = createSagaMiddleware();
  const preloadedState =
    typeof window !== 'undefined'
      ? {
          theme: { current: getPreloadedTheme() },
        }
      : undefined;
  const store = configureStore({
    reducer: rootReducer(),
    preloadedState,
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
