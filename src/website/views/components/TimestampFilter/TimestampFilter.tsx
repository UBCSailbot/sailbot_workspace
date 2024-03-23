import React, { useState } from 'react';
import { connect } from 'react-redux';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { Button, Grid } from '@mui/material';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DatePicker } from '@mui/x-date-pickers/DatePicker';
import styles from './timestampfilter.module.css';

// FIX ELEVATION ISSUE !!!!!!!!!!!!!!!!!!!!!!!!!!!!

function TimestampFilter(props) {
  const [startDate, setStartDate] = useState('');
  const [endDate, setEndDate] = useState('');

  const dispatch = props.dispatch;

  const handleApplyChange = () => {
    let cond1 = startDate != null && endDate != null;
    let cond2 = _parseISOString(endDate) > _parseISOString(startDate);

    if (cond1 == true && cond2 == true) {
      dispatch({
        type: 'TIMESTAMP',
        payload: {
          startDate: startDate.toISOString(),
          endDate: endDate.toISOString(),
        },
      });
    }
  };

  const handleResetChange = () => {
    dispatch({
      type: 'TIMESTAMP',
      payload: {
        startDate: null,
        endDate: null,
      },
    });

    setStartDate(null);
    setEndDate(null);
  };

  const handleStartChange = (newStartDate) => {
    setStartDate(newStartDate);
  };

  const handleEndChange = (newEndDate) => {
    setEndDate(newEndDate);
  };

  function _parseISOString(s: string) {
    return Math.floor(Date.parse(s) / 1000); // Converts to seconds
  }

  return (
    <Accordion className={styles.accordion}>
      <AccordionSummary
        aria-controls='timestampfilter-content'
        id='timestampfilter-header'
      >
        <Typography align='center' sx={{ width: '100%' }}>
          Timestamp Filter
        </Typography>
      </AccordionSummary>
      <AccordionDetails>
        <Grid container rowSpacing={1}>
          <Grid item xs={12}>
            <LocalizationProvider dateAdapter={AdapterDayjs}>
              <DatePicker
                label='Starting Date'
                onChange={handleStartChange}
                slotProps={{
                  field: { clearable: true },
                  popper: {
                    disablePortal: true,
                  },
                }}
              />
              {startDate && props.children}
            </LocalizationProvider>
          </Grid>
          <Grid item xs={12}>
            <LocalizationProvider dateAdapter={AdapterDayjs}>
              <DatePicker
                label='Ending Date'
                onChange={handleEndChange}
                slotProps={{
                  field: { clearable: true },
                  popper: {
                    disablePortal: true,
                  },
                }}
              />
              {endDate && props.children}
            </LocalizationProvider>
          </Grid>
          <Grid item xs={6}>
            <Button variant='contained' onClick={handleApplyChange}>
              APPLY
            </Button>
          </Grid>
          <Grid item xs={6}>
            <Button variant='outlined' onClick={handleResetChange}>
              RESET
            </Button>
          </Grid>
        </Grid>
      </AccordionDetails>
    </Accordion>
  );
}

export default connect()(TimestampFilter);
