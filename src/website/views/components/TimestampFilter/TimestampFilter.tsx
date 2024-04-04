import React, { useState } from 'react';
import { connect } from 'react-redux';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { Button, Grid } from '@mui/material';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DateTimePicker } from '@mui/x-date-pickers/DateTimePicker';
import styles from './timestampfilter.module.css';
import DataFilterActions from '@/stores/DataFilter/DataFilterActions';

function TimestampFilter({ setTimeStamp }) {
  const [startDate, setStartDate] = useState(null);
  const [endDate, setEndDate] = useState(null);

  const handleApplyChange = () => {
    let emptyDate = startDate != null && endDate != null;
    let validDate = parseISOString(endDate) > parseISOString(startDate);

    if (emptyDate == true && validDate == true) {
      setTimeStamp({
        startDate: startDate.toISOString(),
        endDate: endDate.toISOString(),
      });
    }
  };

  const handleResetChange = () => {
    setStartDate(null);
    setEndDate(null);
    setTimeStamp({
      startDate: null,
      endDate: null,
    });
  };

  const handleStartChange = (newStartDate) => {
    setStartDate(newStartDate);
  };

  const handleEndChange = (newEndDate) => {
    setEndDate(newEndDate);
  };

  function parseISOString(s: string) {
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
              />
            </LocalizationProvider>
          </Grid>
          <Grid item xs={12}>
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
              />
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

const mapDispatchToProps = {
  setTimeStamp: (timestamps: any) => {
    return {
      type: DataFilterActions.SET_TIMESTAMP,
      payload: timestamps,
    };
  },
};

export default connect(null, mapDispatchToProps)(TimestampFilter);
