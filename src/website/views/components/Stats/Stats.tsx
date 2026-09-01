'use client';

import styles from './stats.module.css';

interface StatsProps {
  className?: string;
}

const Stats = ({ className }: StatsProps) => {
  return (
    <div className={`${styles.stats} ${className ?? ''}`.trim()}>
      <div className={styles.notice} role='status'>
        <p className={styles.noticeLabel}>Last updated August 31, 2026</p>
        <p className={styles.noticeBody}>
          Research data collected during POLARIS&apos;s voyage will be
          available shortly. We are actively working to prepare it for sharing.
        </p>
      </div>
    </div>
  );
};

export default Stats;
