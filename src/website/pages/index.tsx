import React, { useState } from 'react';
import dynamic from 'next/dynamic';
import { CircularProgress, Modal } from '@mui/material';
import Header from '@/views/components/Header/Header';
import styles from './style.module.css';
import PolarisContainer from '@/views/components/Polaris/PolarisContainer';

const MapsContainer = dynamic(() => import('@/views/MapsContainer'), {
  loading: () => (
    <CircularProgress
      style={{
        position: 'absolute',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
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
  const [isModalOpen, setIsModalOpen] = useState(false);
  const handleOpenModal = () => setIsModalOpen(true);
  const handleCloseModal = () => setIsModalOpen(false);

  return (
    <>
      <Header onInfoButtonClick={handleOpenModal} />
      <div className={styles.mainContainer}>
        <MapsContainer />
        <div className={styles.dashboardContainer}>
          <DashboardContainer />
        </div>
      </div>
      <div className={styles.polarisContainer}>
        <PolarisContainer />
      </div>
      <Modal
        open={isModalOpen}
        onClose={handleCloseModal}
        aria-labelledby="modal-title"
        aria-describedby="modal-description"
        sx={{'z-index': 20000000000}}
      >
<div style={{
  position: 'absolute',
  top: '50%',
  left: '50%',
  transform: 'translate(-50%, -50%)',
  width: '80vw', 
  maxWidth: '800px', 
  backgroundColor: 'white',
  border: '2px solid #000',
  boxShadow: '0 4px 20px rgba(0, 0, 0, 0.1)',
  padding: '20px', 
  overflowY: 'scroll',
  overflowX: 'hidden',
  maxHeight: '90vh',
}}>
  <div style={{ transform: 'scale(1)', transformOrigin: 'top center' }}>
    <PolarisContainer />
  </div>
</div>

      </Modal>
    </>
  );
}

