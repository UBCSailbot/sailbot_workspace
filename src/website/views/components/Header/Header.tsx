'use client';

import Link from 'next/link';
import styles from './header.module.css';
import { usePathname } from 'next/navigation';
import { useTheme } from '@/app/hooks/useTheme';

const Header = () => {
  const currentPath = usePathname();
  const { theme, toggleTheme } = useTheme();

  return (
    <div className={styles.header}>
      <div className={styles.title}>
        <img src='LogoPlainWhite.svg' alt='Logo' className={styles.logo} />
        <h1>SAILBOTPOLARIS.COM</h1>
      </div>
      <div className={styles.links}>
        <Link
          href='/'
          style={currentPath === '/' ? { textDecoration: 'underline' } : {}}
        >
          DASHBOARD
        </Link>
        <Link
          href='/about'
          style={
            currentPath === '/about' ? { textDecoration: 'underline' } : {}
          }
        >
          ABOUT
        </Link>
        <button onClick={toggleTheme}>
          {theme ? 'Light Mode' : 'Dark Mode'}
        </button>
      </div>
    </div>
  );
};

export default Header;
