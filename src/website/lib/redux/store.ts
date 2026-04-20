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
  if (typeof window === 'undefined') {
    return Theme.Dark;
  }

  try {
    const stored = localStorage.getItem('theme');
    if (stored === 'dark') return Theme.Dark;
    if (stored === 'light') return Theme.Light;
  } catch {
    // may error out if user disables localstorage or something. thats fine.
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
