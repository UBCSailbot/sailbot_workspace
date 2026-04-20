'use client';

/* Core */
import { Provider } from 'react-redux';

/* Instruments */
import { reduxStore } from '@/lib/redux';
import { ThemeSyncListener } from '@/lib/ThemeSyncListener';

export const Providers = (props: React.PropsWithChildren) => {
  return (
    <Provider store={reduxStore}>
      <ThemeSyncListener />
      {props.children}
    </Provider>
  );
};
