import React, { useState } from "react";
import { connect } from 'react-redux';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import * as dayjs from 'dayjs'
import { Button, Grid } from '@mui/material'
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DatePicker } from '@mui/x-date-pickers/DatePicker';
import styles from './timestampfilter.module.css'

// FIX ELEVATION ISSUE !!!!!!!!!!!!!!!!!!!!!!!!!!!!

function TimestampFilter(props) {
    dayjs().format()

    const [startDate, setStartDate] = useState("");
    const [endDate, setEndDate] = useState("");

    const dispatch = (props.dispatch)

    const handleApplyChange = () => {
        dispatch({
            type: 'TIMESTAMP',
            payload: {
                startDate: startDate,
                endDate: endDate
            }
        })
    }

    const handleStartChange = (newStartDate) => {
        setStartDate(dayjs(newStartDate.$d).format('YYYY-MM-DD'))
        // dispatch({
        //     type: 'TIMESTAMP',
        //     payload: {
        //         startDate: dayjs(newStartDate.$d).format('YYYY-MM-DD'),
        //         endDate: endDate
        //     }
        // })
        // console.log(newStartDate.$d.toString())
    }

    const handleEndChange = (newEndDate) => {
        setEndDate(dayjs(newEndDate.$d).format('YYYY-MM-DD'))
        // dispatch({
        //     type: 'TIMESTAMP',
        //     payload: {
        //         startDate: startDate,
        //         endDate: dayjs(newEndDate.$d).format('YYYY-MM-DD')
        //     }
        // })
        // console.log(newEndDate.$d.toString())
    }

    return (
        <Accordion
            className={styles.accordion}
        >
            <AccordionSummary
                aria-controls="timestampfilter-content"
                id="timestampfilter-header"
            >
                <Typography align="center" sx={{width: '100%'}}>Timestamp Filter</Typography>
            </AccordionSummary>
            <AccordionDetails>
                <Grid container rowSpacing={1}>
                    <Grid item xs={12}>
                        <LocalizationProvider dateAdapter={AdapterDayjs}>
                            <DatePicker
                                label="Starting Date"
                                onChange={handleStartChange}
                                className = {styles.datepicker}
                            />
                            {startDate && props.children}
                        </LocalizationProvider>
                    </Grid>
                    <Grid item xs={12}>
                        <LocalizationProvider dateAdapter={AdapterDayjs}>
                            <DatePicker
                                label="Ending Date"
                                onChange={handleEndChange}
                                className = {styles.datepicker}
                            />
                            {endDate && props.children}
                        </LocalizationProvider>
                    </Grid>
                    <Grid item xs={12}>
                        <Button
                            variant="contained"
                            onClick={handleApplyChange}
                        >
                            APPLY
                        </Button>
                    </Grid>
                </Grid>
            </AccordionDetails>
        </Accordion>
    )
}

export default connect()(TimestampFilter);
