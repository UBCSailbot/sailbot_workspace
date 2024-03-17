import React, { useState } from "react";
import { connect } from 'react-redux';
import { TextField } from '@mui/material'
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DatePicker } from '@mui/x-date-pickers/DatePicker';

function TimestampFilter(props) {

    const [startingDate, setStartingDate, endingDate, setEndingDate] = useState<Date | null>(null)

    const dispatch = (props.dispatch)

    const handleChange = (newDate) => {
        setStartingDate(newDate)
        // dispatch({type: 'TIMESTAMP', payload: newDate})
        console.log(newDate)
    }

    console.log({ startingDate })

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
                        value={startingDate}
                        onChange={handleChange}
                    />
                    {startingDate && props.children}
                </LocalizationProvider>
                <LocalizationProvider dateAdapter={AdapterDayjs}>
                    <DatePicker
                        label="Ending Date"
                        value={endingDate}
                    />
                    {endingDate && props.children}
                </LocalizationProvider>
            </AccordionDetails>
        </Accordion>
    )
}

export default connect()(TimestampFilter);
