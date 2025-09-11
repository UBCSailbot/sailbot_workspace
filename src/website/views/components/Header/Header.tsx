import Link from 'next/link';
import styles from './header.module.css';
import { useRouter } from 'next/router';

const Header = () => {
  const router = useRouter();
  const currentPath = router.pathname;

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
      </div>
    </div>
  );
};

export default Header;
