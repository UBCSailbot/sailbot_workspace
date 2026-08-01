'use client';

import { useState } from 'react';
import Link from 'next/link';
import styles from './mobileNav.module.css';

type MobileView = 'map' | 'data';

interface MobileNavProps {
  activeView: MobileView;
  onSelectView: (view: MobileView) => void;
}

const MenuIcon = () => (
  <svg
    width='28'
    height='28'
    viewBox='0 0 24 24'
    fill='none'
    stroke='white'
    strokeWidth='2'
  >
    <line x1='3' y1='6' x2='21' y2='6' />
    <line x1='3' y1='12' x2='21' y2='12' />
    <line x1='3' y1='18' x2='21' y2='18' />
  </svg>
);

const CloseIcon = () => (
  <svg
    width='28'
    height='28'
    viewBox='0 0 24 24'
    fill='none'
    stroke='white'
    strokeWidth='2.5'
  >
    <line x1='5' y1='5' x2='19' y2='19' />
    <line x1='19' y1='5' x2='5' y2='19' />
  </svg>
);

const MapIcon = () => (
  <svg width='26' height='26' viewBox='0 0 24 24'>
    <path d='M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5a2.5 2.5 0 110-5 2.5 2.5 0 010 5z' />
  </svg>
);

const DataIcon = () => (
  <svg width='26' height='26' viewBox='0 0 24 24'>
    <rect x='3' y='12' width='4' height='9' />
    <rect x='10' y='7' width='4' height='14' />
    <rect x='17' y='3' width='4' height='18' />
  </svg>
);

const AboutIcon = () => (
  <svg width='26' height='26' viewBox='0 0 24 24'>
    <path d='M12 2a10 10 0 100 20 10 10 0 000-20zm1 15h-2v-6h2v6zm0-8h-2V7h2v2z' />
  </svg>
);

const MobileNav = ({ activeView, onSelectView }: MobileNavProps) => {
  const [open, setOpen] = useState(false);

  const select = (view: MobileView) => {
    onSelectView(view);
    setOpen(false);
  };

  return (
    <>
      <div className={styles.bar}>
        <img src='LogoPlainWhite.svg' alt='Sailbot' className={styles.logo} />
        <button
          className={styles.iconButton}
          aria-label='Open navigation'
          onClick={() => setOpen(true)}
        >
          <MenuIcon />
        </button>
      </div>

      {open && (
        <div className={styles.overlay}>
          <button
            className={`${styles.iconButton} ${styles.closeButton}`}
            aria-label='Close navigation'
            onClick={() => setOpen(false)}
          >
            <CloseIcon />
          </button>

          <nav className={styles.menu}>
            <button
              className={styles.menuItem}
              aria-current={activeView === 'map'}
              onClick={() => select('map')}
            >
              <MapIcon />
              <span>MAP</span>
            </button>
            <button
              className={styles.menuItem}
              aria-current={activeView === 'data'}
              onClick={() => select('data')}
            >
              <DataIcon />
              <span>DATA</span>
            </button>
            <Link
              className={styles.menuItem}
              href='/about'
              onClick={() => setOpen(false)}
            >
              <AboutIcon />
              <span>ABOUT</span>
            </Link>
          </nav>
        </div>
      )}
    </>
  );
};

export default MobileNav;
