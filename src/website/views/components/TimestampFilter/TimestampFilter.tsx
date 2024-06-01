import React, { useState } from 'react';
import { connect } from 'react-redux';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { Button } from '@mui/material';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DateTimePicker } from '@mui/x-date-pickers/DateTimePicker';
import styles from './timestampfilter.module.css';
import DataFilterActions from '@/stores/DataFilter/DataFilterActions';

interface TimestampFilterProps {
  setTimestamp: (timestamps: any) => any;
}

function TimestampFilter({ setTimestamp }: TimestampFilterProps) {
  const [startDate, setStartDate] = useState(null);
  const [endDate, setEndDate] = useState(null);

  const handleApplyChange = () => {
    let emptyDate = startDate != null && endDate != null;
    let validDate = parseISOString(endDate) > parseISOString(startDate);

    if (emptyDate == true && validDate == true) {
      setTimestamp({
        // @ts-ignore
        startDate: startDate.toISOString(),
        // @ts-ignore
        endDate: endDate.toISOString(),
      });
    }
  };

  const handleResetChange = () => {
    setStartDate(null);
    setEndDate(null);
    setTimestamp({
      startDate: null,
      endDate: null,
    });
  };

  const handleStartChange = (newStartDate: any) => {
    setStartDate(newStartDate);
  };

  const handleEndChange = (newEndDate: any) => {
    setEndDate(newEndDate);
  };

  function parseISOString(s: string | null) {
    if (s === null) return 0;
    return Math.floor(Date.parse(s) / 1000); // Converts to seconds
  }

  return (
    <div className={styles.dropdownMenu}>
      <LocalizationProvider dateAdapter={AdapterDayjs}>
        <DateTimePicker
          label='Starting Date'
          value={startDate}
          onChange={handleStartChange}
          slotProps={{
            field: { clearable: true },
            popper: {
              disablePortal: true,
            },
          }}
          className={styles.dropdownItem}
        />
      </LocalizationProvider>
      <LocalizationProvider dateAdapter={AdapterDayjs}>
        <DateTimePicker
          label='Ending Date'
          value={endDate}
          onChange={handleEndChange}
          slotProps={{
            field: { clearable: true },
            popper: {
              disablePortal: true,
            },
          }}
          className={styles.dropdownItem}
        />
      </LocalizationProvider>
      <Button
        variant='contained'
        className={styles.dropdownBtn}
        disableElevation
        onClick={handleApplyChange}
      >
        APPLY
      </Button>
      <Button
        variant='outlined'
        className={styles.dropdownBtn}
        onClick={handleResetChange}
      >
        RESET
      </Button>
    </div>
  );
}

const mapDispatchToProps = {
  setTimestamp: (timestamps: any) => {
    return {
      type: DataFilterActions.SET_TIMESTAMP,
      payload: timestamps,
    };
  },
};

export default connect(null, mapDispatchToProps)(TimestampFilter);
