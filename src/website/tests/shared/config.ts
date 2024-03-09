export const SERVICE_HOST =
  process.env.NEXT_PUBLIC_SERVER_HOST || 'http://localhost';
export const SERVICE_PORT = process.env.NEXT_PUBLIC_SERVER_PORT || 3005;
export const SERVICE_URL = `${SERVICE_HOST}:${SERVICE_PORT}`;
