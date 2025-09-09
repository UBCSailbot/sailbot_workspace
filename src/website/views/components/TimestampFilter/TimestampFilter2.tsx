import React, { useEffect, useState } from 'react';
import { connect } from 'react-redux';
import DatePicker from 'react-datepicker';
import styles from './timeStampFilter2.module.css';
import DataFilterActions from '@/stores/DataFilter/DataFilterActions';
import clsx from 'clsx';
import { DataFilterState } from '@/stores/DataFilter/DataFilterTypes';

interface TimestampFilterProps {
  dataFilter: DataFilterState;
  setTimestamp: (timestamps: any) => any;
}

const TimestampFilter2 = ({
  dataFilter,
  setTimestamp,
}: TimestampFilterProps) => {
  const [startDate, setStartDate] = useState<Date | null>(null);
  const [endDate, setEndDate] = useState<Date | null>(null);

  useEffect(() => {
    if (!startDate || !endDate) return;

    setTimestamp({
      startDate: startDate.toISOString(),
      endDate: endDate.toISOString(),
    });
  }, [startDate, endDate]);

  const handleReset = () => {
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
            selected={
              dataFilter.timestamps.startDate
                ? new Date(dataFilter.timestamps.startDate)
                : startDate
            }
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
            selected={
              dataFilter.timestamps.endDate
                ? new Date(dataFilter.timestamps.endDate)
                : endDate
            }
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
        <div className={styles.reset} onClick={handleReset}>
          Reset
        </div>
      </div>
    </div>
  );
};

const mapStateToProps = (state: any) => ({
  dataFilter: state.dataFilter,
});

const mapDispatchToProps = {
  setTimestamp: (timestamps: any) => ({
    type: DataFilterActions.SET_TIMESTAMP,
    payload: timestamps,
  }),
};

export default connect(mapStateToProps, mapDispatchToProps)(TimestampFilter2);
