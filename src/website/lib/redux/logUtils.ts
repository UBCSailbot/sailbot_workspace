const isDevelopment = process.env.NODE_ENV === 'development';

export const clearLogsPeriodically = () => {
  if (isDevelopment) {
    setInterval(() => {
      console.clear();
    }, 60000); // Clear logs every minute
  }
};
