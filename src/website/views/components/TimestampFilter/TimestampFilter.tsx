import React, { useState } from "react";
import { connect } from 'react-redux';
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DatePicker } from '@mui/x-date-pickers/DatePicker';

function TimestampFilter(props) {
    let startingDateStr = "", endingDateStr = ""

    const dispatch = (props.dispatch)

    const handleStartChange = (newStartDate) => {
        dispatch({type: 'TIMESTAMP', payload: [newStartDate.$d.toString(), endingDateStr]})
        console.log(newStartDate.$d.toString())
        startingDateStr = newStartDate.$d.toString()
    }

    const handleEndChange = (newEndDate) => {
        dispatch({type: 'TIMESTAMP', payload: [startingDateStr, newEndDate.$d.toString()]})
        console.log(newEndDate.$d.toString())
        endingDateStr = newEndDate.$d.toString()
    }

    return (
        <Accordion>
            <AccordionSummary
                aria-controls="timestampfilter-content"
                id="timestampfilter-header"
            >
                <Typography>Timestamp Filter</Typography>
            </AccordionSummary>
            <AccordionDetails>
                <LocalizationProvider dateAdapter={AdapterDayjs}>
                    <DatePicker
                        label="Starting Date"
                        onChange={handleStartChange}
                    />
                    {props.children}
                </LocalizationProvider>
                <LocalizationProvider dateAdapter={AdapterDayjs}>
                    <DatePicker
                        label="Ending Date"
                        onChange={handleEndChange}
                    />
                    {props.children}
                </LocalizationProvider>
            </AccordionDetails>
        </Accordion>
    )
}

export default connect()(TimestampFilter);
