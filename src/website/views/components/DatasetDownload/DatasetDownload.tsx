'use client';

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
import { describe } from 'node:test';

const dataSets = [
  {
    title: 'GPS',
    data: ['GPS', '12 hours'],
    action: downloadGPSData,
    description: 'Download GPS data for the last 12 hours.',
  },
  {
    title: 'AIS Ships',
    data: ['AIS Ships', '12 hours'],
    action: downloadAISShipsData,
    description: 'Download AIS Ships data for the last 12 hours.',
  },
  {
    title: 'Global Path',
    data: ['Global Path', '12 hours'],
    action: downloadGlobalPathData,
    description: 'Download Global Path data for the last 12 hours.',
  },
  {
    title: 'Local Path',
    data: ['Local Path', '12 hours'],
    action: downloadLocalPathData,
    description: 'Download Local Path data for the last 12 hours.',
  },
  {
    title: 'Batteries',
    data: ['Batteries', '12 hours'],
    action: downloadBatteriesData,
    description: 'Download Batteries data for the last 12 hours.',
  },
  {
    title: 'Wind Sensors',
    data: ['Wind Sensors', '12 hours'],
    action: downloadWindSensorsData,
    description: 'Download Wind Sensors data for the last 12 hours.',
  },
  {
    title: 'Generic Sensors',
    data: ['Generic Sensors', '12 hours'],
    action: downloadGenericSensorsData,
    description: 'Download Generic Sensors data for the last 12 hours.',
  },
];

export const DatasetDownload = () => {
  return (
    <div className={styles.dataSetDownload}>
      {dataSets.map((dataSet, index) => (
        <div key={index} className={styles.dataSetCard}>
          <div className={styles.title}>{dataSet.title}</div>
          <div className={styles.description}>{dataSet.description}</div>
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
