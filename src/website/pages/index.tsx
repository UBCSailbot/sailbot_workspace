import dynamic from 'next/dynamic';
import { CircularProgress } from '@mui/material';
import Header from '@/views/components/Header/Header';
import styles from './style.module.css';
import PolarisContainer from '@/views/components/Polaris/PolarisContainer';

const MapsContainer = dynamic(() => import('@/views/MapsContainer'), {
  loading: () => (
    <CircularProgress
      style={{
        position: 'absolute',
        top: 0,
        bottom: 0,
        left: 0,
        right: 0,
        margin: 'auto',
        height: '100px',
        width: '100px',
      }}
    />
  ),
  ssr: false,
});

const DashboardContainer = dynamic(() => import('@/views/DashboardContainer'), {
  ssr: false,
}); 

export default function Home() {
  return (
    <>
      <Header />
      <div className={styles.mainContainer}>
        <MapsContainer />
        <div className={styles.dashboardContainer}>
          <DashboardContainer />
        </div>
      </div>
      <div className={styles.polarisContainer}>
        <PolarisContainer />
      </div>
    </>
  );
}
