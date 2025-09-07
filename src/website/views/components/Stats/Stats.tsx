import dynamic from 'next/dynamic';
import styles from './stats.module.css';
import HistoricDataDropdown from './HistoricDataDropdown';
import RearrangeGraphDropdown from './RearrangeGraphDropdown';

const LineChart = dynamic(() => import('../LineChart/LineChart'), {
  ssr: false,
});

const data = [
  new Float64Array([1696118400, 1696204800, 1696291200]),
  new Float64Array([10, 15, 8]),
];

const Stats = () => {
  return (
    <div className={styles.stats}>
      <div className={styles.heading}>
        <div className={styles.title}>POLARIS IS CURRENTLY:</div>
        <div className={styles.summary}>
          12.5 KNOTS | 278.3° | 37.7749° N, 122.4194° W
        </div>
        <div className={styles.toolbar}>
          <HistoricDataDropdown />
          <RearrangeGraphDropdown />
        </div>
      </div>
      <div className={styles.lineCharts}>
        <LineChart data={data} />
        <LineChart data={data} />
        <LineChart data={data} />
        <LineChart data={data} />
      </div>
    </div>
  );
};

export default Stats;
