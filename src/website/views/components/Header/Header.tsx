'use client';

import Link from 'next/link';
import styles from './header.module.css';
import { usePathname } from 'next/navigation';
import { useDispatch, useSelector } from 'react-redux';
import { toggleTheme, selectTheme, Theme } from '@/lib/redux/theme/themeSlice';
import LightModeIcon from '@/public/icons/light_mode.svg';
import DarkModeIcon from '@/public/icons/dark_mode.svg';
import LogoPlainBlack from '@/public/LogoPlainBlack.svg';
import LogoPlainWhite from '@/public/LogoPlainWhite.svg';
import { useEffect, useState } from 'react';

const Header = () => {
  const currentPath = usePathname();
  const dispatch = useDispatch();
  const theme = useSelector(selectTheme);

  const [hasMounted, setHasMounted] = useState(false);

  useEffect(() => {
    setHasMounted(true);
  }, []);

  return (
    <div className={styles.header}>
      <div className={styles.title}>
        {hasMounted ? (
          theme === Theme.Light ? (
            <LogoPlainBlack width={50} height={50} />
          ) : (
            <LogoPlainWhite width={50} height={50} />
          )
        ) : (
          <div style={{ width: 50, height: 50 }} />
        )}
        <h1>SAILBOTPOLARIS.COM</h1>
      </div>
      <div className={styles.right}>
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
        <div
          onClick={() => dispatch(toggleTheme())}
          className={styles.themeToggle}
        >
          {theme === Theme.Light ? (
            <DarkModeIcon width={30} height={30} />
          ) : (
            <LightModeIcon width={30} height={30} />
          )}
        </div>
      </div>
    </div>
  );
};

export default Header;
