import React from 'react';
import { Providers } from '@/lib/providers';
import type { AppProps } from 'next/app';
import '../styles/globals.css';
import './uplot.css';

export default function App({ Component, pageProps }: AppProps) {
  return (
    <Providers>
      <Component {...pageProps} />
    </Providers>
  );
}
