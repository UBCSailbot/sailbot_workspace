'use client';

import Link from 'next/link';
import styles from './header.module.css';
import { usePathname } from 'next/navigation';

const Header = () => {
  const currentPath = usePathname();

  return (
    <div className={styles.header}>
      <div className={styles.title}>
        <img src='LogoPlainWhite.svg' alt='Logo' className={styles.logo} />
        <h1>testtest123</h1>
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
      </div>
    </div>
  );
};

export default Header;
