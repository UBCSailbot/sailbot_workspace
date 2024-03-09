import React from 'react';
import styles from './header.module.css';

function Header() {
  return (
    <header className={styles.header}>
      <img src='SailbotLogo.png' alt='Logo' className={styles.logo} />
      <h1 className={styles.title}>UBC SAILBOT</h1>
    </header>
  );
}

export default Header;
