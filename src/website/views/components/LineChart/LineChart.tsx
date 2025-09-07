'use client';

import { useState, useLayoutEffect, useRef } from 'react';
import UplotReact from 'uplot-react';
import StatsDropdown from '../Stats/StatsDropdown';
import DownloadIcon from '@/public/icons/download.svg';

import styles from './lineChartStyles.module.css';

interface LineChartProps {
  data: any[];
}

const LineChart = ({ data }: LineChartProps) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(0);

  useLayoutEffect(() => {
    if (!containerRef.current) return;

    setWidth(containerRef.current.offsetWidth);

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        setWidth(entry.contentRect.width);
      }
    });

    resizeObserver.observe(containerRef.current);

    return () => resizeObserver.disconnect();
  }, []);

  const options = {
    // when resizing the window, there may be a vertical scrollbar.
    // this is likely because react "lags behind" when rerendering multiple times quickly
    // if we resize the window too quickly, react might be holding on to a stale width value
    // so we just multiply by 0.99 to make sure the actual size is ever so slightly smaller
    // so even if we are using a stale width, it should still fit.
    width: width * 0.99,
    height: 300,
    axes: [
      {
        font: "12px 'Fira Code', 'SF Mono', Consolas, 'Courier New', monospace",
        stroke: '#ffffff',
        grid: {
          stroke: '#6b6b6b',
          width: 0.5,
        },
      },
      {
        font: "12px 'Fira Code', 'SF Mono', Consolas, 'Courier New', monospace",
        stroke: '#ffffff',
        grid: {
          stroke: '#6b6b6b',
          width: 0.5,
        },
      },
    ],
    series: [
      {
        label: 'Time',
      },
      {
        label: 'Value',
        stroke: 'white',
        width: 2,
      },
    ],
  };

  return (
    <div className={styles.lineChart}>
      <div className={styles.header}>
        <h3 className={styles.title}>BATTERIES</h3>
        <div className={styles.downloadOptions}>
          <div className={styles.downloadButton}>
            <DownloadIcon />
            csv
          </div>
          <div className={styles.downloadButton}>
            <DownloadIcon />
            xlsx
          </div>
        </div>
      </div>
      <div ref={containerRef}>
        {width ? (
          <UplotReact options={options} data={data} />
        ) : (
          <div className={styles.placeholder} />
        )}
      </div>
    </div>
  );
};

export default LineChart;
