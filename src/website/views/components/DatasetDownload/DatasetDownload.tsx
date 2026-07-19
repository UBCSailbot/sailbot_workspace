'use client';

import { useEffect, useState } from 'react';
import {
  downloadGPSData,
  downloadAISShipsData,
  downloadGlobalPathData,
  downloadLocalPathData,
  downloadBatteriesData,
  downloadWindSensorsData,
  downloadTempSensorsData,
  downloadPhSensorsData,
  downloadSalinitySensorsData,
} from '@/utils/DownloadData';
import DownloadIcon from '@/public/icons/download.svg';
import styles from './datasetDownload.module.css';

type LastUpdatedMap = Record<string, string | null>;

type DataSet = {
  key: string;
  title: string;
  description: string;
  action?: (format: string) => void;
  externalUrl?: string;
};

// TODO(team): replace with the real Drive link to the hydrophone recordings
const HYDROPHONE_DRIVE_URL = 'https://drive.google.com/';

const dataSets: DataSet[] = [
  {
    key: 'GPS',
    title: 'GPS',
    action: downloadGPSData,
    description: 'Vessel position, velocity, and heading measurements.',
  },
  {
    key: 'AIS',
    title: 'AIS Ships',
    action: downloadAISShipsData,
    description: 'Position and size of nearby AIS ships.',
  },
  {
    key: 'GlobalPath',
    title: 'Global Path',
    action: downloadGlobalPathData,
    description: 'Global vessel latitude and longitude path.',
  },
  {
    key: 'LocalPath',
    title: 'Local Path',
    action: downloadLocalPathData,
    description: 'Local vessel latitude and longitude path.',
  },
  {
    key: 'Batteries',
    title: 'Batteries',
    action: downloadBatteriesData,
    description: 'Battery voltage and current measurements.',
  },
  {
    key: 'WindSensors',
    title: 'Wind Sensors',
    action: downloadWindSensorsData,
    description: 'Wind speed and direction measurements.',
  },
  {
    key: 'TempSensors',
    title: 'Temperature',
    action: downloadTempSensorsData,
    description: 'Water temperature measurements (°C).',
  },
  {
    key: 'PhSensors',
    title: 'pH',
    action: downloadPhSensorsData,
    description: 'Water pH measurements.',
  },
  {
    key: 'SalinitySensors',
    title: 'Salinity',
    action: downloadSalinitySensorsData,
    description: 'Water conductivity measurements (µS/cm).',
  },
  {
    key: 'Hydrophone',
    title: 'Hydrophone',
    externalUrl: HYDROPHONE_DRIVE_URL,
    description:
      'Raw hydrophone audio recordings.'
  },
];

const formatTimestamp = (iso: string) => {
  return iso.split('T')[0];
};

export const DatasetDownload = () => {
  // Redux selector to get last updated timestamp
  const [lastUpdated, setLastUpdated] = useState<LastUpdatedMap>({});

  useEffect(() => {
    (async () => {
      const res = await fetch('/api/datasets');
      const data = await res.json();

      setLastUpdated(data.data);
    })();
  }, []);

  return (
    <div className={styles.dataSetDownload}>
      {dataSets.map((dataSet, index) => {
        const lastUpdatedTimestamp = lastUpdated[dataSet.key];
        return (
          <div key={index} className={styles.dataSetCard}>
            <div>
              <div className={styles.title}>{dataSet.title}</div>
              <div className={styles.description}>{dataSet.description}</div>
            </div>
            {dataSet.externalUrl ? (
              <div>
                <div className={styles.downloadOptions}>
                  <a
                    className={styles.downloadButton}
                    href={dataSet.externalUrl}
                    target='_blank'
                    rel='noopener noreferrer'
                  >
                    <DownloadIcon />
                    Open in Drive
                  </a>
                </div>
                <p className={styles.dateUpdated}>Stored externally</p>
              </div>
            ) : (
              <div>
                <div className={styles.downloadOptions}>
                  <div
                    className={styles.downloadButton}
                    onClick={() => dataSet.action?.('CSV')}
                  >
                    <DownloadIcon />
                    CSV
                  </div>
                  <div
                    className={styles.downloadButton}
                    onClick={() => dataSet.action?.('XLSX')}
                  >
                    <DownloadIcon />
                    XLSX
                  </div>
                  <div
                    className={styles.downloadButton}
                    onClick={() => dataSet.action?.('JSON')}
                  >
                    <DownloadIcon />
                    JSON
                  </div>
                </div>
                <p className={styles.dateUpdated}>
                  Last Updated:{' '}
                  {lastUpdatedTimestamp
                    ? `${formatTimestamp(lastUpdatedTimestamp)}`
                    : 'NO DATA'}
                </p>
              </div>
            )}
          </div>
        );
      })}
    </div>
  );
};
