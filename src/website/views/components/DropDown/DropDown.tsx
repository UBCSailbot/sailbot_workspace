import React from 'react';
import { Accordion, AccordionSummary, AccordionDetails } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';

function CustomAccordion({ title, children }) {
  return (
    <Accordion sx={{
      backgroundColor: '#ecf0f1',
      border: '1px solid #bdc3c7',
      boxShadow: '0px 2px 4px -1px rgb(0 0 0 / 20%), 0px 4px 5px 0px rgb(0 0 0 / 14%), 0px 1px 10px 0px rgb(0 0 0 / 12%)',
      '&:before': {
        backgroundColor: 'transparent',
      },
    }}>
      <AccordionSummary
        expandIcon={<ExpandMoreIcon />}
        aria-controls="panel1-content"
        id="panel1-header"
        sx={{
          backgroundColor: '#3498db',
          color: 'white',
          '& .MuiAccordionSummary-expandIconWrapper.Mui-expanded': {
            transform: 'rotate(180deg)',
          },
          '&.Mui-expanded': {
            minHeight: 48,
            backgroundColor: '#2980b9',
          },
          '&:hover': {
            backgroundColor: '#2980b9',
          },
        }}
      >
        {title}
      </AccordionSummary>
      <AccordionDetails sx={{
        backgroundColor: '#bdc3c7',
      }}>
        {children}
      </AccordionDetails>
    </Accordion>
  );
}

export default CustomAccordion;
