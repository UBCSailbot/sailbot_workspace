'use client';

// very strange component.
// it doesnt return anything, we just need this to be a component because we need to use react hooks

import { useEffect } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { setTheme, selectTheme, Theme } from '@/lib/redux/theme/themeSlice';

export function ThemeSyncListener() {
  const dispatch = useDispatch();
  const theme = useSelector(selectTheme);

  useEffect(() => {
    document.documentElement.classList.toggle('dark', theme === Theme.Dark);
    localStorage.setItem('theme', theme);
  }, [theme]);

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
