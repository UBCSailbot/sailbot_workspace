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
    },
    toggleTheme: (state) => {
      state.current =
        state.current === Theme.Light ? Theme.Dark : Theme.Light;
    },
  },
});

export const { setTheme, toggleTheme } = themeSlice.actions;

export default themeSlice.reducer;

export const selectTheme = (state: { theme: ThemeState }) =>
  state.theme.current;
