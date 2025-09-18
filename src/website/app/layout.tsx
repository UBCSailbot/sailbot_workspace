import React from 'react';
import { Providers } from '@/lib/providers';
import '../styles/globals.css';
import '../styles/uplot.css';
import 'react-datepicker/dist/react-datepicker.css';

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang='en'>
      <body>
        <Providers>{children}</Providers>
      </body>
    </html>
  );
}