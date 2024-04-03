import React from 'react';
import styles from './header.module.css';
import DropdownBtn from '@/views/components/DropDown/DropDown';

function Header() {
  return (
    <header className={styles.header}>
      <div className={styles.headerContainer}>
        <img src='Sailbot Logo Plain (white).png' alt='Logo' className={styles.logo} />
        <h1 className={styles.title}>UBC SAILBOT</h1>
      </div>
      <DropdownBtn />
    </header>
  );
}



export default Header;
