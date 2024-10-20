import React, { useState } from 'react';
import { Accordion, AccordionSummary, AccordionDetails } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import styles from './dataset.module.css';
import Select from 'react-select'

interface DatasetProps {
  title: string;
  content: string[];
  downloadAction: () => any;
}

const Dataset = ({ title, content, downloadAction }: DatasetProps) => {
  const titles = ['DATA TYPE', 'TIMESCALE', 'FILE', 'ACCESS'];
  const fileTypes = [
    { value: 'JSON', label: 'JSON' },
    { value: 'CSV', label: 'CSV' },
    { value: 'XLSX', label: 'EXCEL' },
  ];

  const [selectedFileType, setSelectedFileType] = useState('JSON');

  return (
    <Accordion className={styles.accordionCustom}>
      <AccordionSummary
        expandIcon={<ExpandMoreIcon />}
        aria-controls='panel1-content'
        id='panel1-header'
        className={styles.accordionSummaryCustom}
      >
        {title}
      </AccordionSummary>
      <AccordionDetails className={styles.accordionDetailsCustom}>
        <div className={styles.detailsFlexContainer}>
          {titles.map((title, index) => (
            <div className={styles.flexItemContainer} key={`title-${index}`}>
              {title}
            </div>
          ))}
        </div>
        <div className={styles.contentContainer}>
          {content.slice(0, 2).map((item, index) => (
            <div
              className={styles.flexItemContainer}
              key={`content-item-${index}`}
            >
              {item}
            </div>
          ))}
          <div className={styles.flexItemContainer}>
            <Select options={fileTypes} defaultValue={fileTypes[0]} onChange={(fileType: { value: string, label: string }) => setSelectedFileType(fileType.value)}/>
          </div>
          <div className={styles.flexItemContainer}>
            <span
              style={{ cursor: 'pointer', textDecoration: 'underline' }}
              onClick={() => downloadAction(selectedFileType)}
            >
              Download
            </span>
          </div>
        </div>
      </AccordionDetails>
    </Accordion>
  );
};

export default Dataset;
