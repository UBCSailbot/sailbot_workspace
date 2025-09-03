import styles from './header.module.css';

const Header = () => {
  return (
    <div className={styles.header}>
      <div className={styles.title}>
        <img src='LogoPlainWhite.svg' alt='Logo' className={styles.logo} />
        <h1>POLARISTRACKER.COM</h1>
      </div>
      <div className={styles.links}>
        <h2>LINK</h2>
        <h2>LINK</h2>
        <h2>LINK</h2>
      </div>
    </div>
  );
};

export default Header;
