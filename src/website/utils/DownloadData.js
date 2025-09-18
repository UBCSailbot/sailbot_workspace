const downloadBlob = async (blob, name) => {
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = name;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
};

// used for exporting when our json is in the format we use for uplot
export const downloadDataFromJSON = async (data, name, format, seriesNames) => {
  try {
    let blob;
    let filename = `${name}.${format.toLowerCase()}`;

    if (format === 'JSON') {
      const jsonStr = JSON.stringify(data);
      blob = new Blob([jsonStr], { type: 'application/json' });
    } else if (format === 'CSV') {
      const headers = seriesNames;
      const rows = [headers];

      for (let i = 0; i < data[0].length; i++) {
        const row = [
          data[0][i],
          ...data.slice(1).map((series) => series[i] ?? ''),
        ];
        rows.push(row);
      }

      const csvContent = rows.map((row) => row.join(',')).join('\n');
      blob = new Blob([csvContent], { type: 'text/csv' });
    } else if (format === 'XLSX') {
      const ExcelJS = require('exceljs');
      const workbook = new ExcelJS.Workbook();
      const worksheet = workbook.addWorksheet('Data');

      const headers = seriesNames;
      worksheet.columns = headers.map((header) => ({
        header,
        key: header,
        width: 15,
      }));

      for (let i = 0; i < data[0].length; i++) {
        const rowData = {};
        headers.forEach((header, j) => {
          rowData[header] = j === 0 ? data[0][i] : data[j][i] ?? null;
        });
        worksheet.addRow(rowData);
      }

      const buffer = await workbook.xlsx.writeBuffer();
      blob = new Blob([buffer], {
        type: 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',
      });
    }

    if (!blob) {
      throw new Error(`Unsupported format: ${format}`);
    }

    downloadBlob(blob, filename);
  } catch (error) {
    console.error('Failed to download data:', error);
  }
};

const downloadDataFromAPI = async (sensorType, format) => {
  try {
    if (!format) {
      return;
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
      const csvString = formatCsv(data, sensorType);
      blob = new Blob([csvString], { type: 'text/csv' });
    } else if (format === 'JSON') {
      const jsonStr = JSON.stringify(data);
      blob = new Blob([jsonStr], { type: 'application/json' });
    } else if (format === 'XLSX') {
      const xlsxBuffer = await formatXLSX(data, sensorType);
      blob = new Blob([new Uint8Array(xlsxBuffer)], {
        type: 'application/vnd.openxmlformats-officedocument.spreadsheetml.sheet',
      });
    }

    downloadBlob(blob, filename);
  } catch (error) {
    console.error('Failed to download data:', error);
  }
};

const formatCsv = (data, sensorType) => {
  switch (sensorType) {
    case 'gps':
      return format2dCsv(data);
    case 'aisships':
    case 'globalpath':
    case 'localpath':
    case 'batteries':
    case 'wind-sensors':
    case 'generic-sensors':
      return format3dCsv(data);
    default:
      throw new Error(`Unsupported sensor type: ${sensorType}`);
  }
};

// helper functions based on data dimensionality
// format3dCsv is built for a very specific data structure

const format2dCsv = (data) => {
  if (!data) return [];
  const headers = ['timestamp', ...Object.keys(data[0]).slice(0, -1)];
  const csvRows = [];
  csvRows.push(headers);

  data.forEach((reading) => {
    const csvRowString = headers.map((header) => reading[header]);
    csvRows.push(csvRowString);
  });

  const csvString = csvRows.join('\n');
  return csvString;
};

const format3dCsv = (data) => {
  const firstItem = data[0];
  if (!firstItem) return [];

  const innerProperties = Object.keys(firstItem);
  const innerHeaders = Object.keys(firstItem[innerProperties[0]][0]);
  const headers = [innerProperties[1], ...innerHeaders];
  const csvRows = [];
  csvRows.push(headers);

  data.forEach((reading) => {
    reading[innerProperties[0]].forEach((innerItem, index) => {
      const timestamp = index === 0 ? reading.timestamp : '';
      const innerHeaderValues = innerHeaders.map(
        (innerHeader) => innerItem[innerHeader],
      );
      const innerRow = [timestamp, ...innerHeaderValues];
      csvRows.push(innerRow);
    });
  });

  const csvString = csvRows.join('\n');
  return csvString;
};

const formatXLSX = async (data, sensorType) => {
  const ExcelJS = require('exceljs');
  const workbook = new ExcelJS.Workbook();
  const worksheet = workbook.addWorksheet('Sheet 1');

  switch (sensorType) {
    case 'gps':
      format2dXLSX(data, worksheet, workbook);
      break;
    case 'aisships':
    case 'globalpath':
    case 'localpath':
    case 'batteries':
    case 'wind-sensors':
    case 'generic-sensors':
      format3dXLSX(data, worksheet, workbook);
      break;
    default:
      throw new Error(`Unsupported sensor type: ${sensorType}`);
  }

  const excelBuffer = await workbook.xlsx.writeBuffer();
  return excelBuffer;
};

const format2dXLSX = (data, worksheet) => {
  const headers = ['timestamp', ...Object.keys(data[0]).slice(0, -1)];
  const columns = headers.map((header) => ({
    header: header,
    key: header,
    width: 30,
  }));
  worksheet.columns = columns;

  data.forEach((reading) => {
    const rowObject = {};
    headers.forEach((header) => (rowObject[header] = reading[header]));
    worksheet.addRow(rowObject);
  });
};

const format3dXLSX = (data, worksheet) => {
  const firstItem = data[0];
  if (!firstItem) return [];

  const innerProperties = Object.keys(firstItem);
  const innerHeaders = Object.keys(firstItem[innerProperties[0]][0]);
  const headers = [innerProperties[1], ...innerHeaders];
  const columns = headers.map((header) => ({
    header: header,
    key: header,
    width: 30,
  }));
  worksheet.columns = columns;

  data.forEach((reading) => {
    reading[innerProperties[0]].forEach((innerItem, index) => {
      const rowObject = {};
      const timestamp = index === 0 ? reading.timestamp : '';
      rowObject['timestamp'] = timestamp;

      innerHeaders.forEach(
        (innerHeader) => (rowObject[innerHeader] = innerItem[innerHeader]),
      );
      worksheet.addRow(rowObject);
    });
  });
};

export const downloadGPSData = (format) => downloadDataFromAPI('gps', format);
export const downloadAISShipsData = (format) =>
  downloadDataFromAPI('aisships', format);
export const downloadGlobalPathData = (format) =>
  downloadDataFromAPI('globalpath', format);
export const downloadLocalPathData = (format) =>
  downloadDataFromAPI('localpath', format);
export const downloadBatteriesData = (format) =>
  downloadDataFromAPI('batteries', format);
export const downloadWindSensorsData = (format) =>
  downloadDataFromAPI('wind-sensors', format);
export const downloadGenericSensorsData = (format) =>
  downloadDataFromAPI('generic-sensors', format);
