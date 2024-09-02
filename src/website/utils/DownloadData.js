const downloadSensorData = async (sensorType, format) => {
  try {
    if(!format) {
      return
    }

    const apiUrl = `${process.env.NEXT_PUBLIC_SERVER_HOST}:${process.env.NEXT_PUBLIC_SERVER_PORT}/api/${sensorType}`;
    const response = await fetch(apiUrl);
    if (!response.ok) {
      throw new Error(`Error: ${response.statusText}`);
    }
    const { data } = await response.json();

    let blob;
    let filename = `${sensorType}.${format}`;

    if (format === 'CSV') {
      const csvRows = [];
      const headers = Object.keys(data[0]);
      csvRows.push(headers.join(','));

      for (const row of data) {
        const values = headers.map(header => {
          const escaped = ('' + row[header]).replace(/"/g, '\\"');
          return `"${escaped}"`;
        });
        csvRows.push(values.join(','));
      }

      const csvString = csvRows.join('\n');
      blob = new Blob([csvString], { type: 'text/csv' });
    } else if (format === 'JSON') {
      const jsonStr = JSON.stringify(data);
      blob = new Blob([jsonStr], { type: 'application/json' });
    }

    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = filename;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
  } catch (error) {
    console.error('Failed to download data:', error);
  }
};

export const downloadGPSData = (format) => downloadSensorData('gps', format);
export const downloadAISShipsData = (format) => downloadSensorData('aisships', format);
export const downloadGlobalPathData = (format) => downloadSensorData('globalpath', format);
export const downloadLocalPathData = (format) => downloadSensorData('localpath', format);
export const downloadBatteriesData = (format) => downloadSensorData('batteries', format);
export const downloadWindSensorsData = (format) => downloadSensorData('wind-sensors', format);
export const downloadGenericSensorsData = (format) =>
  downloadSensorData('generic-sensors', format);
