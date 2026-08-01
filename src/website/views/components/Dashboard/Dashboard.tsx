'use client';

import { useState } from 'react';
import Map from '../Map/Map';
import Stats from '../Stats/Stats';
import MobileNav from '../MobileNav/MobileNav';
import styles from './dashboard.module.css';

const Dashboard = () => {
  const [mobileView, setMobileView] = useState<'map' | 'data'>('map');

  return (
    <div className={styles.dashboard}>
      <MobileNav activeView={mobileView} onSelectView={setMobileView} />
      <Map className={mobileView === 'map' ? '' : styles.hiddenOnMobile} />
      <Stats className={mobileView === 'data' ? '' : styles.hiddenOnMobile} />
    </div>
  );
};

export default Dashboard;
