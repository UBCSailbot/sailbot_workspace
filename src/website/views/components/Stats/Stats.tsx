import HistoryIcon from '@/public/icons/history.svg';
import RearrangeIcon from '@/public/icons/format_line_spacing.svg';
import LineChart from '../LineChart/LineChart';
import styles from './stats.module.css';

const data = [
  new Float64Array([1696118400, 1696204800, 1696291200]),
  new Float64Array([10, 15, 8]),
];

const Stats = () => {
  return (
    <div className={styles.stats}>
      <div className={styles.heading}>
        <div className={styles.title}>POLARIS IS CURRENTLY: </div>
        <div className={styles.summary}>
          12.5 KNOTS | 278.3° | 37.7749° N, 122.4194° W
        </div>
        <div className={styles.toolbar}>
          <div className={styles.toolbarItem}>
            <HistoryIcon />
            View Historic Data
          </div>
          <div className={styles.toolbarItem}>
            <RearrangeIcon />
            Rearrange Graphs
          </div>
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
