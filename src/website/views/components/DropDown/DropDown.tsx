import React, { useState } from 'react';
import styles from './dropdown.module.css';
import { connect } from 'react-redux';

function DropdownBtn(props) {

  const [open, setOpen] = useState(false);

  const dispatch = (props.dispatch)

  const handleClick = () => {
    setOpen(!open)
    dispatch({type: 'REARRANGE_GRAPHS', payload: ["1", "2", "4", "3"]})
  }
  return (
    <>
      <div className={styles.iconButton} onClick={handleClick}> clickToDrop! </div>
      { open && props.children }
    </>
  );
}

export default connect()(DropdownBtn);
