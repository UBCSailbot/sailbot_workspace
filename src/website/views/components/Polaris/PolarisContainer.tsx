import React, {useState, useEffect} from 'react';
import { connect } from 'react-redux';
import styles from '@/views/components/Polaris/PolarisContainer.module.css';
import {
  downloadAISShipsData,
  downloadBatteriesData,
  downloadGPSData,
  downloadGenericSensorsData,
  downloadGlobalPathData,
  downloadLocalPathData,
  downloadWindSensorsData,
} from '@/utils/DownloadData.js';
import Dataset from '@/views/components/Dataset/Dataset';

const PolarisContainer = () => {

  function stringToParagraphs(text: string) {
    return text.split('\\n').map((item, key) => (
      <span key={key}>
        {item}
        <br />
        <br />
      </span>
    ));
  }

  const [description, setDescription] = useState<React.ReactNode>()

  useEffect(() => {
    fetch('/info/PolarisVoyageDescription.txt')
      .then((res) => res.text())
      .then((text) => stringToParagraphs(text))
      .then((paragraphs) => setDescription(paragraphs));
  }, []);

  const customContents = [
    {
      title: 'GPS',
      data: ['GPS', '12 hours', 'JSON'],
      action: downloadGPSData,
    },
    {
      title: 'AIS Ships',
      data: ['AIS Ships', '12 hours', 'JSON'],
      action: downloadAISShipsData,
    },
    {
      title: 'Global Path',
      data: ['Global Path', '12 hours', 'JSON'],
      action: downloadGlobalPathData,
    },
    {
      title: 'Local Path',
      data: ['Local Path', '12 hours', 'JSON'],
      action: downloadLocalPathData,
    },
    {
      title: 'Batteries',
      data: ['Batteries', '12 hours', 'JSON'],
      action: downloadBatteriesData,
    },
    {
      title: 'Wind Sensors',
      data: ['Wind Sensors', '12 hours', 'JSON'],
      action: downloadWindSensorsData,
    },
    {
      title: 'Generic Sensors',
      data: ['Generic Sensors', '12 hours', 'JSON'],
      action: downloadGenericSensorsData,
    },
  ];

  return (
    <div className={styles.container}>
      <div className={styles.spatialCoverageContainer}>
        <div className={styles.spatialCoverageTitle}>Polaris</div>
        <div className={styles.coverageItem}>
          <div>Latitudes</div>
          <div className={styles.coverageContent}>Hello1</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Longitudes</div>
          <div className={styles.coverageContent}>Hello2</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Date Range</div>
          <div className={styles.coverageContent}>Hello3</div>
        </div>
        <div className={styles.coverageItem}>
          <div>Geography</div>
          <div className={styles.coverageContent}>Hello4</div>
        </div>
      </div>
      {description}
      <div className={styles.dataSetsContainer}>
        <div className={styles.dataSetHeader}>Access Data Sets</div>
        {customContents.map((content) => (
          <div className={styles.accordionWrapper} key={content.title}>
            <Dataset
              title={content.title}
              content={content.data}
              downloadAction={content.action}
            />
          </div>
        ))}
      </div>
    </div>
  );
}

export default connect()(PolarisContainer);
