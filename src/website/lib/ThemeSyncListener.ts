'use client';

// very strange component.
// it doesnt return anything, we just need this to be a component because we need to use react hooks

import { useEffect } from 'react';
import { useDispatch } from 'react-redux';
import { setTheme, Theme } from '@/lib/redux/theme/themeSlice';

export function ThemeSyncListener() {
  const dispatch = useDispatch();

  useEffect(() => {
    const handleStorageChange = (e: StorageEvent) => {
      if (e.key === 'theme' && e.newValue) {
        const newTheme = e.newValue === 'dark' ? Theme.Dark : Theme.Light;
        dispatch(setTheme(newTheme));
      }
    };

    window.addEventListener('storage', handleStorageChange);

    return () => {
      window.removeEventListener('storage', handleStorageChange);
    };
  }, [dispatch]);

  return null;
}
