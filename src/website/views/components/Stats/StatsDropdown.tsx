import { useState } from 'react';
import styles from './statsDropdown.module.css';

// includes the dropdown button as well
const StatsDropdown = ({
  button,
  menu,
}: {
  button: React.ReactNode;
  menu: React.ReactNode;
}) => {
  const [isOpen, setIsOpen] = useState(false);

  const handleClick = () => setIsOpen(!isOpen);

  return (
    <div className={styles.dropdown}>
      <div className={styles.button} onClick={handleClick}>
        {button}
      </div>
      {isOpen && <div className={styles.menu}>{menu}</div>}
    </div>
  );
};

export default StatsDropdown;
