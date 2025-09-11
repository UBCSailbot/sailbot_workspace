import Map from '../Map/Map';
import styles from './dashboard.module.css';
import dynamic from 'next/dynamic';

// Stats needs to be rendered client side
const StatsClientSide = dynamic(() => import('../Stats/Stats'), {
  ssr: false,
});

const Dashboard = () => {
  return (
    <div className={styles.dashboard}>
      <Map />
      <StatsClientSide />
    </div>
  );
};

export default Dashboard;
