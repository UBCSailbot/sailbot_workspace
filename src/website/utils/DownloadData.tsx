const downloadSensorData = async (sensorType) => {
  try {
    const apiUrl = `${process.env.NEXT_PUBLIC_SERVER_HOST}:${process.env.NEXT_PUBLIC_SERVER_PORT}/api/${sensorType}`;
    const response = await fetch(apiUrl);
    if (!response.ok) {
      throw new Error(`Error: ${response.statusText}`);
    }
    const { data } = await response.json();
    const jsonStr = JSON.stringify(data);
    const blob = new Blob([jsonStr], { type: 'application/json' });

    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = `${sensorType}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  } catch (error) {
    console.error('Failed to download data:', error);
  }
};

export const downloadGPSData = () => downloadSensorData('gps');
export const downloadAISShipsData = () => downloadSensorData('aisships');
export const downloadGlobalPathData = () => downloadSensorData('globalpath');
export const downloadLocalPathData = () => downloadSensorData('localpath');
export const downloadBatteriesData = () => downloadSensorData('batteries');
export const downloadWindSensorsData = () => downloadSensorData('wind-sensors');
export const downloadGenericSensorsData = () =>
  downloadSensorData('generic-sensors');
