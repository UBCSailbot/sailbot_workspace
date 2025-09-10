import Link from 'next/link';
import styles from './header.module.css';

const Header = () => {
  return (
    <div className={styles.header}>
      <div className={styles.title}>
        <img src='LogoPlainWhite.svg' alt='Logo' className={styles.logo} />
        <h1>SAILBOTPOLARIS.COM</h1>
      </div>
      <div className={styles.links}>
        <Link href='/'>DASHBOARD</Link>
        <Link href='/about'>ABOUT</Link>
      </div>
    </div>
  );
};

export default Header;
