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
    const [startDate, setStartDate] = useState("");
    const [endDate, setEndDate] = useState("");

    const dispatch = (props.dispatch)

    const handleStartChange = (newStartDate) => {
        setStartDate(newStartDate.$d.toString())
        dispatch({type: 'TIMESTAMP', payload: [newStartDate.$d.toString(), endDate]})
        console.log(newStartDate.$d.toString())
    }

    const handleEndChange = (newEndDate) => {
        setEndDate(newEndDate.$d.toString())
        dispatch({type: 'TIMESTAMP', payload: [startDate, newEndDate.$d.toString()]})
        console.log(newEndDate.$d.toString())
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
                    {startDate && props.children}
                </LocalizationProvider>
                <LocalizationProvider dateAdapter={AdapterDayjs}>
                    <DatePicker
                        label="Ending Date"
                        onChange={handleEndChange}
                    />
                    {endDate && props.children}
                </LocalizationProvider>
            </AccordionDetails>
        </Accordion>
    )
}

export default connect()(TimestampFilter);
