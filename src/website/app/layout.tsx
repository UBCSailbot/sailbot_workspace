import React from 'react';
import { Providers } from '@/lib/providers';
import { SET_THEME } from './themeNoFlashScript';
import '@/styles/variables.css';
import '@/styles/globals.css';

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang='en'>
      <head>
        <script dangerouslySetInnerHTML={{ __html: SET_THEME }} />
      </head>
      <body>
        <Providers>{children}</Providers>
      </body>
    </html>
  );
}
