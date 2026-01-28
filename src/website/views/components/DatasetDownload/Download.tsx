'use client';

import { useEffect, useRef } from 'react';
import { DatasetDownload } from '../DatasetDownload/DatasetDownload';
import styles from './datasetDownload.module.css';
import gsap from 'gsap';
import { useSelector } from 'react-redux';
import type { GPSState } from '@/stores/GPS/GPSTypes';

const formatTimestamp = (iso: string) => {
  return iso.replace('T', ' ').split('.')[0];
};

const Download = () => {
  // GSAP animation context
  const containerRef = useRef<HTMLDivElement>(null);

  // Redux selector to get last updated timestamp
  const gps = useSelector((state: any) => state.gps as GPSState);
  const lastTimestamp = gps.data.length
    ? gps.data[gps.data.length - 1].timestamp
    : null;

  useEffect(() => {
    if (!containerRef.current || !containerRef.current.children.length) return;

    const ctx = gsap.context(() => {
      const query = gsap.utils.selector(containerRef);

      gsap.from(query('*'), {
        opacity: 0,
        y: 30,
        duration: 0.6,
        ease: 'power2.out',
        stagger: 0.0075,
      });
    }, containerRef);

    return () => ctx.revert();
  }, []);

  return (
    <div ref={containerRef} className={styles.downloadContainer}>
      <div className={styles.downloadHeader}>
        <h2>DOWNLOAD OUR DATASETS</h2>
        <p suppressHydrationWarning>
          {/* Last Updated: {lastTimestamp ? `${formatTimestamp(lastTimestamp)} PST (UTC-8)` : "NO DATA"} */}
        </p>
      </div>
      <DatasetDownload />
    </div>
  );
};

export default Download;
