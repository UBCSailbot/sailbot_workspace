import { createSlice, PayloadAction } from '@reduxjs/toolkit';

export enum Theme {
  Light = 'light',
  Dark = 'dark',
}

interface ThemeState {
  current: Theme;
}

export const themeSlice = createSlice({
  name: 'theme',
  initialState: {
    current: Theme.Dark,
  },
  reducers: {
    setTheme: (state, action: PayloadAction<Theme>) => {
      state.current = action.payload;
      document.documentElement.classList.toggle(
        'dark',
        action.payload === Theme.Dark,
      );
      localStorage.setItem(
        'theme',
        action.payload === Theme.Dark ? 'dark' : 'light',
      );
    },
    toggleTheme: (state) => {
      const newTheme = state.current === Theme.Light ? Theme.Dark : Theme.Light;
      state.current = newTheme;
      document.documentElement.classList.toggle(
        'dark',
        newTheme === Theme.Dark,
      );
      localStorage.setItem('theme', newTheme === Theme.Dark ? 'dark' : 'light');
    },
  },
});

export const { setTheme, toggleTheme } = themeSlice.actions;

export default themeSlice.reducer;

export const selectTheme = (state: { theme: ThemeState }) =>
  state.theme.current;
