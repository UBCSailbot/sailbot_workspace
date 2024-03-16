import React from "react";
import Accordion from '@mui/material/Accordion';
import AccordionSummary from '@mui/material/AccordionSummary';
import AccordionDetails from '@mui/material/AccordionDetails';
import Typography from '@mui/material/Typography';
import { AdapterDayjs } from '@mui/x-date-pickers/AdapterDayjs';
import { LocalizationProvider } from '@mui/x-date-pickers/LocalizationProvider';
import { DatePicker } from '@mui/x-date-pickers/DatePicker';

class TimestampFilter extends React.Component<TimestampFilter>{

    render() {
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
                        />
                    </LocalizationProvider>
                    <LocalizationProvider dateAdapter={AdapterDayjs}>
                        <DatePicker
                            label="Ending Date"
                        />
                    </LocalizationProvider>
                </AccordionDetails>
            </Accordion>
        )
    }
}

export default TimestampFilter;
