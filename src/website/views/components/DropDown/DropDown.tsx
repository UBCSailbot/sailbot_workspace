import React from 'react';
import { Accordion, AccordionSummary, AccordionDetails } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { styled } from '@mui/material/styles';
import '@/views/components/DropDown/DropDown.module.css'

function CustomAccordion({ title, content, downloadAction }) {
  const titles = ["DATA TYPE", "TIMESCALE", "FILE", "ACCESS"];
  return (
    <Accordion className="accordion-custom">
      <AccordionSummary
        expandIcon={<ExpandMoreIcon />}
        aria-controls="panel1-content"
        id="panel1-header"
        className="accordion-summary-custom"
      >
        {title}
      </AccordionSummary>
      <AccordionDetails className="accordion-details-custom">
        <div className="details-flex-container">
          {titles.map((title, index) => (
            <div className="flex-item-container" key={`title-${index}`}>{title}</div>
          ))}
        </div>
        <div className="content-container">
          {content.slice(0, 3).map((item, index) => (
            <div className="flex-item-container" key={`content-item-${index}`}>{item}</div>
          ))}
          <div className="flex-item-container">
            <span style={{ cursor: 'pointer', textDecoration: 'underline' }} onClick={downloadAction}>Download</span>
          </div>
        </div>
      </AccordionDetails>
    </Accordion>
  );
}

export default CustomAccordion;