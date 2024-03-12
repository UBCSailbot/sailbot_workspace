import React from 'react';
import { Accordion, AccordionSummary, AccordionDetails } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import { styled } from '@mui/material/styles';

const FlexItemContainer = styled('div')({
  flex: 1,
  display: 'flex',
  justifyContent: 'center', 
});

const DetailsFlexContainer = styled('div')({
  display: 'flex',
  justifyContent: 'space-between',
  width: '100%',
  position: 'relative', 
  paddingBottom: '8px',
  marginTop: '2px',
  '&::after': { 
    content: '""', 
    display: 'block',
    position: 'absolute',
    bottom: '0', 
    left: '0', 
    width: '100%', 
    height: '1px', 
    backgroundColor: 'black', 
  },
});

const ContentContainer = styled('div')({
  display: 'flex',
  justifyContent: 'space-between',
  width: '100%',
  paddingTop: '8px',
});

function CustomAccordion({ title, content, downloadAction }) {
  const titles = ["DATA TYPE", "TIMESCALE", "FILE", "ACCESS"];
  return (
    <Accordion sx={{
      backgroundColor: '#ecf0f1',
      border: '1px solid #bdc3c7',
      boxShadow: '0px 2px 4px -1px rgb(0 0 0 / 20%), 0px 4px 5px 0px rgb(0 0 0 / 14%), 0px 1px 10px 0px rgb(0 0 0 / 12%)',
    }}>
      <AccordionSummary
        expandIcon={<ExpandMoreIcon />}
        aria-controls="panel1-content"
        id="panel1-header"
        sx={{ backgroundColor: '#3498db', color: 'white', }}
      >
        {title}
      </AccordionSummary>
      <AccordionDetails sx={{ backgroundColor: '#bdc3c7', flexDirection: 'column', padding: '16px 16px', }}>
        <DetailsFlexContainer> 
          {titles.map((title, index) => (
            <FlexItemContainer key={`title-${index}`}>{title}</FlexItemContainer>
          ))}
        </DetailsFlexContainer>
        <ContentContainer>
          {content.slice(0, 3).map((item, index) => (
            <FlexItemContainer key={`content-item-${index}`}>{item}</FlexItemContainer>
          ))}
          <FlexItemContainer>
            <span style={{ cursor: 'pointer', textDecoration: 'underline' }} onClick={downloadAction}>Download</span>
          </FlexItemContainer>
        </ContentContainer>
      </AccordionDetails>
    </Accordion>
  );
}


export default CustomAccordion;
