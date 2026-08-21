'use client';

import styles from './stats.module.css';

interface StatsProps {
  className?: string;
}

const Stats = ({ className }: StatsProps) => {
  return (
    <div className={`${styles.stats} ${className ?? ''}`.trim()}>
      <div className={styles.notice} role='status'>
        <p className={styles.noticeLabel}>Temporary during voyage</p>
        <p className={styles.noticeBody}>
          POLARIS is collecting data in the Pacific. To save satellite credits,
          we only share internal diagnostics and basic data while it&apos;s
          underway. Full research datasets will be uploaded once POLARIS is
          back on land.
        </p>
      </div>
    </div>
  );
};

export default Stats;
