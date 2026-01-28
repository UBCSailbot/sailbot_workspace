'use client';

import { useEffect, useRef } from 'react';
import { DatasetDownload } from '../DatasetDownload/DatasetDownload';
import styles from './datasetDownload.module.css';
import gsap from 'gsap';

const Download = () => {
  const containerRef = useRef<HTMLDivElement>(null); // container ref for GSAP animations

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
      </div>
      <DatasetDownload />
    </div>
  );
};

export default Download;
