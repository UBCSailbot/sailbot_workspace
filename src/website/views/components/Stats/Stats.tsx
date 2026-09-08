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
          POLARIS is back home after successfully completing its first
          autonomous ocean voyage. Research data collected throughout the
          journey will be available shortly. We have improvements planned and
          are actively working toward our next mission.
        </p>
      </div>
    </div>
  );
};

export default Stats;
