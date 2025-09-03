import Stats from '../Stats/Stats';
import Map from '../Map/Map';
import styles from './dashboard.module.css';

const Dashboard = () => {
  return (
    <div className={styles.dashboard}>
      <Map />
      <Stats />
    </div>
  );
};

export default Dashboard;
