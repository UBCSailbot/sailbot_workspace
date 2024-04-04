import React, { useState } from 'react';
import dynamic from 'next/dynamic';
import { CircularProgress, Modal } from '@mui/material';
import Header from '@/views/components/Header/Header';
import TimestampFilter from '@/views/components/TimestampFilter/TimestampFilter';
import styles from './style.module.css';
import PolarisContainer from '@/views/components/Polaris/PolarisContainer';

const MapsContainer = dynamic(() => import('@/views/MapsContainer'), {
  loading: () => <CircularProgress className={styles.loadingSpinner} />,
  ssr: false,
});

const DashboardContainer = dynamic(() => import('@/views/DashboardContainer'), {
  ssr: false,
});

export default function Home() {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const handleOpenModal = () => setIsModalOpen(true);
  const handleCloseModal = () => setIsModalOpen(false);

  return (
    <>
      <div className={styles.parent}>
        <Header />
        <div className={styles.topRight}>
          <TimestampFilter />
        </div>
      </div>
      <div className={styles.maincontainer}>
        <MapsContainer />
        <div className={styles.dashboardContainer}>
          <DashboardContainer />
        </div>
      </div>

      <Modal
        open={isModalOpen}
        onClose={handleCloseModal}
        aria-labelledby='modal-title'
        aria-describedby='modal-description'
        sx={{ 'z-index': 20000000000 }}
      >
        <div className={styles.modalContent}>
          <div style={{ transform: 'scale(1)', transformOrigin: 'top center' }}>
            <PolarisContainer />
          </div>
        </div>
      </Modal>
    </>
  );
}
