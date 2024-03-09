import React, { useState } from 'react';
import styles from './dropdown.module.css';

function DropdownMenu() {

  function DropdownItem(props) {
    return (
      <div className={styles.dropdownItem}>
        {props.children}
      </div>
    )
  }

  let graphArray = [];

  for (let i = 1; i < 5; i++) {
    graphArray.push(
      <DropdownItem key={i}>
        {i}
      </DropdownItem>
    )
  }

  let graphsOrderArray = [];
  for (let i = 0; i < 4; i++) {
    graphsOrderArray.push(graphArray[i].key)
  }
  const [order, setOrder] = useState(graphsOrderArray);

  console.log(graphsOrderArray)

  return (
    <div className={styles.dropdownMenu}>
      {graphArray}
      <button onClick={() => setOrder(graphsOrderArray)}>
        set
      </button>
    </div>
  )
}

export default DropdownMenu;
