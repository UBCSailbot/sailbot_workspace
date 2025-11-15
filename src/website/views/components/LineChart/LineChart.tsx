'use client';

import { useState, useLayoutEffect, useRef } from 'react';
import UplotReact from 'uplot-react';
import DownloadIcon from '@/public/icons/download.svg';
import { downloadDataFromJSON } from '@/utils/DownloadData';
import { useSelector } from 'react-redux';
import { selectTheme, Theme } from '@/lib/redux/theme/themeSlice';
import styles from './lineChartStyles.module.css';
import './customUplot.css';

interface SeriesData {
  label: string;
  unit?: string;
  stroke?: string;
}

// change this later fr
interface LineChartProps {
  data: any[];
  title: string;
  seriesData: SeriesData[];
}

const LineChart = ({ data, title, seriesData }: LineChartProps) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(0);
  const theme = useSelector(selectTheme);

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
        stroke: theme === Theme.Dark ? '#ffffff' : '#000000',
        grid: {
          stroke: '#6b6b6b',
          width: 0.5,
        },
      },
      {
        font: "12px 'Fira Code', 'SF Mono', Consolas, 'Courier New', monospace",
        stroke: theme === Theme.Dark ? '#ffffff' : '#000000',
        grid: {
          stroke: '#6b6b6b',
          width: 0.5,
        },
      },
    ],
    series: [
      ...seriesData.map((series) => ({
        label: series.label,
        stroke: series.stroke,   
        width: 2,
        value: series.unit
          ? (_u: any, v: any) => (v === null ? '--' : `${v} ${series.unit}`)
          : undefined,
      })),
    ],
  };

  const seriesNames = seriesData.map((series) => series.label);

  const download = (format: string) => {
    downloadDataFromJSON(data, title, format, seriesNames);
  };

  return (
    <div className={styles.lineChart}>
      <div className={styles.header}>
        <h3 className={styles.title}>{title}</h3>
        <div className={styles.downloadOptions}>
          <div
            className={styles.downloadButton}
            onClick={() => download('CSV')}
          >
            <DownloadIcon />
            CSV
          </div>
          <div
            className={styles.downloadButton}
            onClick={() => download('XLSX')}
          >
            <DownloadIcon />
            XLSX
          </div>
          <div
            className={styles.downloadButton}
            onClick={() => download('JSON')}
          >
            <DownloadIcon />
            JSON
          </div>
        </div>
      </div>
      <div ref={containerRef}>
        {width ? (
          data[0].length === 0 ? (
            <div className={styles.noData}>No data to display</div>
          ) : (
            <UplotReact options={options} data={data} />
          )
        ) : (
          <div className={styles.placeholder} />
        )}
      </div>
    </div>
  );
};

export default LineChart;
