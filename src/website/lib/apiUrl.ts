export const getApiUrl = (endpoint: string): string => {
  if (typeof window !== 'undefined') {
    const isProduction = window.location.hostname !== 'localhost';
    if (isProduction) {
      return endpoint;
    }
  }

  const host = process.env.NEXT_PUBLIC_SERVER_HOST || 'http://localhost';
  const port = process.env.NEXT_PUBLIC_SERVER_PORT || '3005';
  const cleanHost = host.replace(/\/$/, '');

  return `${cleanHost}:${port}${endpoint}`;
};
