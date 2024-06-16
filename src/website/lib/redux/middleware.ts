import * as Redux from 'redux';
import { createLogger } from 'redux-logger';

const middleware: Redux.Middleware[] = [
  createLogger({
    duration: true,
    timestamp: true,
    collapsed: true,
    colors: {
      title: () => '#139BFE',
      prevState: () => '#1C5FAF',
      action: () => '#149945',
      nextState: () => '#A47104',
      error: () => '#ff0005',
    },
    predicate: () => typeof window !== 'undefined',
  }),
];

export { middleware };
