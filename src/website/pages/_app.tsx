import React from 'react';
import { Providers } from '@/lib/providers';
import type { AppProps } from 'next/app';
import '../styles/globals.css';
import '../styles/uplot.css';
import 'react-datepicker/dist/react-datepicker.css';

export default function App({ Component, pageProps }: AppProps) {
  return (
    <Providers>
      <Component {...pageProps} />
    </Providers>
  );
}
