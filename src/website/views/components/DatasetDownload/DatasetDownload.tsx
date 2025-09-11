import {
  downloadGPSData,
  downloadAISShipsData,
  downloadGlobalPathData,
  downloadLocalPathData,
  downloadBatteriesData,
  downloadWindSensorsData,
  downloadGenericSensorsData,
} from '@/utils/DownloadData';
import DownloadIcon from '@/public/icons/download.svg';
import styles from './datasetDownload.module.css';

const dataSets = [
  {
    title: 'GPS',
    data: ['GPS', '12 hours'],
    action: downloadGPSData,
  },
  {
    title: 'AIS Ships',
    data: ['AIS Ships', '12 hours'],
    action: downloadAISShipsData,
  },
  {
    title: 'Global Path',
    data: ['Global Path', '12 hours'],
    action: downloadGlobalPathData,
  },
  {
    title: 'Local Path',
    data: ['Local Path', '12 hours'],
    action: downloadLocalPathData,
  },
  {
    title: 'Batteries',
    data: ['Batteries', '12 hours'],
    action: downloadBatteriesData,
  },
  {
    title: 'Wind Sensors',
    data: ['Wind Sensors', '12 hours'],
    action: downloadWindSensorsData,
  },
  {
    title: 'Generic Sensors',
    data: ['Generic Sensors', '12 hours'],
    action: downloadGenericSensorsData,
  },
];

export const DatasetDownload = () => {
  return (
    <div className={styles.dataSetDownload}>
      {dataSets.map((dataSet, index) => (
        <div key={index} className={styles.dataSetCard}>
          <div className={styles.title}>{dataSet.title}</div>
          <div className={styles.downloadOptions}>
            <div
              className={styles.downloadButton}
              onClick={() => dataSet.action('CSV')}
            >
              <DownloadIcon />
              CSV
            </div>
            <div
              className={styles.downloadButton}
              onClick={() => dataSet.action('XLSX')}
            >
              <DownloadIcon />
              XLSX
            </div>
            <div
              className={styles.downloadButton}
              onClick={() => dataSet.action('JSON')}
            >
              <DownloadIcon />
              JSON
            </div>
          </div>
        </div>
      ))}
    </div>
  );
};
