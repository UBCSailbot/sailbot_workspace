export const clearLogsPeriodically = () => {
  setInterval(() => {
    console.clear();
  }, 60000); // Clear logs every minute
};
