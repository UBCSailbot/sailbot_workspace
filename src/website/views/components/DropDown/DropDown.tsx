import React, { useState } from 'react';
import styles from './dropdown.module.css';
import DropdownMenu from '@/views/components/DropDown/DropdownMenu';

const DropdownBtn = () => {

  const [showMenu, setShowMenu] = useState(false);

  const handleClick = () => {
    setShowMenu(!showMenu)
  }

  return (
    <div className="menuContainer">
      <div className={ showMenu? styles.iconButtonOpen : styles.iconButton } onClick={ handleClick }>
      { showMenu? "Drag and Drop to Sort!" : "Sort Graphs" }
      </div>
      <div className={ showMenu? styles.dropdownActive : styles.dropdownInactive }>
        <DropdownMenu />
      </div>
    </div>
  );
}

export default (DropdownBtn);
