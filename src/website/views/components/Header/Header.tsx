import React from 'react';
import styles from './header.module.css';
import DropdownBtn from '../Dropdown/Dropdown';
import DropdownMenu from '../Dropdown/DropdownMenu';

function Header() {
  return (
    <header className={styles.header}>
      <div className={styles.headerContainer}>
        <img src='SailbotLogo.png' alt='Logo' className={styles.logo} />
        <h1 className={styles.title}>UBC SAILBOT</h1>
      </div>
      <DropdownBtn>
        <DropdownMenu />
      </DropdownBtn>
    </header>
  );
}



export default Header;
