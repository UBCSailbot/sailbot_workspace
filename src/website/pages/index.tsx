import React, { useState, useEffect } from 'react';
import dynamic from 'next/dynamic';
import { CircularProgress, Modal } from '@mui/material';
import Header from '@/views/components/Header/Header';
import styles from './style.module.css';
import PolarisContainer from '@/views/components/Polaris/PolarisContainer';
import { clearLogsPeriodically } from '@/lib/redux/logUtils';

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

  useEffect(() => {
    clearLogsPeriodically();
  }, []);

  return (
    <>
      <Header onInfoButtonClick={handleOpenModal} />
      <div className={styles.mainContainer}>
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
