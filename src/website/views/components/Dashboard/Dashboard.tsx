'use client';

import { useState, useEffect } from 'react';
import Map from '../Map/Map';
import Stats from '../Stats/Stats';
import MobileNav from '../MobileNav/MobileNav';
import styles from './dashboard.module.css';

const Dashboard = () => {
  const [mobileView, setMobileView] = useState<'map' | 'data'>('map');

  // Let the mobile nav deep-link to the data pane (e.g. DATA from /about).
  useEffect(() => {
    if (window.location.hash === '#data') {
      setMobileView('data');
    }
  }, []);

  return (
    <div className={styles.dashboard}>
      <MobileNav activeView={mobileView} onSelectView={setMobileView} />
      <Map className={mobileView === 'map' ? '' : styles.hiddenOnMobile} />
      <Stats className={mobileView === 'data' ? '' : styles.hiddenOnMobile} />
    </div>
  );
};

export default Dashboard;
