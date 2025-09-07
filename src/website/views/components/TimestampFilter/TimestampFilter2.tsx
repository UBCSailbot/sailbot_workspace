import React, { useState } from 'react';
import { connect } from 'react-redux';
import DatePicker from 'react-datepicker';
import styles from './timeStampFilter2.module.css';
import DataFilterActions from '@/stores/DataFilter/DataFilterActions';
import clsx from 'clsx';

interface TimestampFilterProps {
  setTimestamp: (timestamps: any) => any;
}

const TimestampFilter2 = ({ setTimestamp }: TimestampFilterProps) => {
  const [startDate, setStartDate] = useState<Date | null>(null);
  const [endDate, setEndDate] = useState<Date | null>(null);

  const handleApplyChange = () => {
    if (!startDate || !endDate) return;
    if (endDate <= startDate) return;

    setTimestamp({
      startDate: startDate.toISOString(),
      endDate: endDate.toISOString(),
    });
  };

  const handleResetChange = () => {
    setStartDate(null);
    setEndDate(null);
    setTimestamp({
      startDate: null,
      endDate: null,
    });
  };

  return (
    <div className={styles.container}>
      <div className={clsx(styles.inputs, styles.calendarWrapper)}>
        <div className={styles.dateGroup}>
          <label>FROM:</label>
          <DatePicker
            selected={startDate}
            onChange={(date) => setStartDate(date)}
            showTimeSelect
            timeFormat='HH:mm'
            timeIntervals={15}
            dateFormat='MMMM d, yyyy h:mm aa'
            placeholderText='MMMM d, yyyy h:mm aa'
            className={styles.input}
            calendarClassName={styles.calendar}
          />
        </div>
        <div className={styles.dateGroup}>
          <label>TO:</label>
          <DatePicker
            selected={endDate}
            onChange={(date) => setEndDate(date)}
            showTimeSelect
            timeFormat='HH:mm'
            timeIntervals={15}
            dateFormat='MMMM d, yyyy h:mm aa'
            placeholderText='MMMM d, yyyy h:mm aa'
            minDate={startDate ?? undefined}
            className={styles.input}
            calendarClassName={styles.calendar}
          />
        </div>
      </div>
    </div>
  );
};

const mapDispatchToProps = {
  setTimestamp: (timestamps: any) => ({
    type: DataFilterActions.SET_TIMESTAMP,
    payload: timestamps,
  }),
};

export default connect(null, mapDispatchToProps)(TimestampFilter2);
