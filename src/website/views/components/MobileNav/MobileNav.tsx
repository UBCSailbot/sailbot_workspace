'use client';

import { useState, ReactNode } from 'react';
import Link from 'next/link';
import { usePathname } from 'next/navigation';
import styles from './mobileNav.module.css';

type MobileView = 'map' | 'data';

interface MobileNavProps {
  // Provided on the dashboard, where MAP/DATA toggle the visible pane. Omitted
  // elsewhere (e.g. /about), where MAP/DATA navigate back to the dashboard.
  activeView?: MobileView;
  onSelectView?: (view: MobileView) => void;
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

const DownloadIcon = () => (
  <svg width='26' height='26' viewBox='0 0 24 24'>
    <path d='M11 4h2v7h3l-4 4-4-4h3zM5 18h14v2H5z' />
  </svg>
);

const MobileNav = ({ activeView, onSelectView }: MobileNavProps) => {
  const [open, setOpen] = useState(false);
  const pathname = usePathname();
  const close = () => setOpen(false);

  const select = (view: MobileView) => {
    onSelectView?.(view);
    close();
  };

  // On the dashboard MAP/DATA switch the pane in place; anywhere else they are
  // links back to the dashboard (DATA lands on the data pane via #data).
  const viewItem = (
    view: MobileView,
    icon: ReactNode,
    label: string,
    href: string,
  ) =>
    onSelectView ? (
      <button
        className={styles.menuItem}
        aria-current={activeView === view}
        onClick={() => select(view)}
      >
        {icon}
        <span>{label}</span>
      </button>
    ) : (
      <Link className={styles.menuItem} href={href} onClick={close}>
        {icon}
        <span>{label}</span>
      </Link>
    );

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
            onClick={close}
          >
            <CloseIcon />
          </button>

          <nav className={styles.menu}>
            {viewItem('map', <MapIcon />, 'MAP', '/')}
            {viewItem('data', <DataIcon />, 'DATA', '/#data')}
            <Link
              className={styles.menuItem}
              href='/about'
              aria-current={pathname === '/about'}
              onClick={close}
            >
              <AboutIcon />
              <span>ABOUT</span>
            </Link>
            <Link
              className={styles.menuItem}
              href='/download'
              aria-current={pathname === '/download'}
              onClick={close}
            >
              <DownloadIcon />
              <span>DOWNLOAD</span>
            </Link>
          </nav>
        </div>
      )}
    </>
  );
};

export default MobileNav;
