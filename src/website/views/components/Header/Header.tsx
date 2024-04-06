import React from 'react';
import { IconButton } from '@mui/material';
import InfoIcon from '@mui/icons-material/Info';
import styles from './header.module.css';
import DropdownBtn from '@/views/components/DropDown/DropDown';
import TimestampFilter from '../TimestampFilter/TimestampFilter';
import TimestampBtn from '../TimestampFilter/TimestampBtn';

interface HeaderProps {
  onInfoButtonClick: () => void;
}

function Header({ onInfoButtonClick }: HeaderProps) {
  return (
    <header className={styles.header}>
      <div className={styles.headerContainer}>
        <img
          src='Sailbot Logo Plain (white).png'
          alt='Logo'
          className={styles.logo}
        />
        <h1 className={styles.title}>UBC SAILBOT</h1>
      </div>
      <div className={styles.infoButton}>
        <TimestampBtn />
        <DropdownBtn />
        <IconButton onClick={onInfoButtonClick} color='inherit'>
          <InfoIcon />
        </IconButton>
      </div>
    </header>
  );
}

export default Header;
