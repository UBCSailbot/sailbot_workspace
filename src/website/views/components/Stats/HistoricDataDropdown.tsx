'use client';

import { useState, useEffect, useRef } from 'react';
import HistoryIcon from '@/public/icons/history.svg';
import styles from './stats.module.css';
import dynamic from 'next/dynamic';

const TimestampFilter = dynamic(
  () => import('@/views/components/TimestampFilter/TimestampFilter'),
);

const HistoricDataDropdown = () => {
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        dropdownRef.current &&
        !dropdownRef.current.contains(event.target as Node)
      ) {
        setIsOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleClick = () => setIsOpen(!isOpen);

  return (
    <div ref={dropdownRef}>
      <div className={styles.dropdownButton} onClick={handleClick}>
        <HistoryIcon />
        View Historic Data
      </div>
      {isOpen && (
        <div className={styles.dropdownMenu}>
          <TimestampFilter />
        </div>
      )}
    </div>
  );
};

export default HistoricDataDropdown;
